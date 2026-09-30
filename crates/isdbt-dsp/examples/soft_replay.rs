//! ダンプされた soft ビット列から Viterbi 以降をオフラインで再生し、
//! 保存された RS 入力バイト列と一致するかを検証する。
//!
//! 目的: 「soft は健全なのに RS が壊れる」原因が
//! - Viterbi が soft を正しく復��していない（Viterbi の問題）
//! - それ以前の段（depunc / byte 整列）が soft を壊している（上流の問題）
//! のどちらかを判定すること。
//!
//! 使い方:
//!   soft_replay <soft.txt> <vit.txt> <byte.hex>
//! soft.txt: '0'/'1'/'.' の並び（post-depuncture、量子化済み）
//! vit.txt : Viterbi 出力ビット列（'0'/'1'）
//! byte.hex: RS 入力 204 バイト（descramble 後）
use isdbt_dsp::viterbi::ViterbiStreaming;
use isdbt_dsp::ts::{ByteDeinterleaver, BI_I, BI_M};
use isdbt_dsp::rs;

const BYTE_LATENCY: usize = BI_M * BI_I * (BI_I - 1);

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() < 3 {
        eprintln!("usage: soft_replay <soft.txt> <vit.txt> <byte.hex>");
        return;
    }
    // 入力が f32 ならそれを使う（量子化版なら文字fallenち）
    let raw0 = std::fs::read_to_string(&a[0]).unwrap();
    let soft_f32: Option<Vec<f32>> = if raw0.trim_start().chars().next()
        .map(|c| c.is_ascii_digit() || c == '-' || c == '+' || c == '.').unwrap_or(false)
        && raw0.contains(' ') {
        Some(raw0.split_whitespace().filter_map(|x| x.parse::<f32>().ok()).collect())
    } else { None };
    let soft: Vec<char> = if soft_f32.is_some() { Vec::new() } else {
        raw0.chars().filter(|c| *c != '\n' && *c != '\r').collect()
    };
    let vit_s: Vec<char> = std::fs::read_to_string(&a[1]).unwrap().chars()
        .filter(|c| *c != '\n' && *c != '\r').collect();
    let hex: String = std::fs::read_to_string(&a[2]).unwrap().chars()
        .filter(|c| c.is_ascii_hexdigit()).collect();
    let mut target = Vec::new();
    let hb = hex.as_bytes();
    let mut i = 0;
    while i + 1 < hb.len() {
        target.push(u8::from_str_radix(&hex[i..i + 2], 16).unwrap());
        i += 2;
    }

    // --- 1. soft から Viterbi を再実行して vit を再現できるか ---
    let sv_ref: &[f32] = soft_f32.as_deref().unwrap_or(&[]);
    let n_chars = if soft_f32.is_some() { sv_ref.len() } else { soft.len() };
    let n = n_chars - (n_chars % 2);
    let mut vs = ViterbiStreaming::new(96);
    let mut replayed: Vec<u8> = Vec::new();
    let mut t = 0;
    while 2 * t + 1 < n {
        let (fx, fy) = if !sv_ref.is_empty() {
            (sv_ref[2 * t], sv_ref[2 * t + 1])
        } else {
            let fv = |c: char| if c == '.' { 0.0 } else if c == '1' { 1.0 } else { -1.0 };
            (fv(soft[2 * t]), fv(soft[2 * t + 1]))
        };
        if let Some(b) = vs.push(fx, fy) {
            replayed.push(b & 1);
        }
        t += 1;
    }
    println!("soft {} 値 (f32={}) -> {} ステップ -> {} 出力ビット (元 vit {})",
        n, soft_f32.is_some(), t, replayed.len(), vit_s.len());

    // --- 2. 再実行した vit が 保存済み vit と一致するか ---
    // 両者の**末尾**は同一のブロックに対応するので、末尾から比較する。
    // 先頭から比較すると ring の巻き戻り位置が違うため一致しない。
    let rv: Vec<u8> = replayed.iter().map(|b| if *b != 0 { b'1' } else { b'0' }).collect();
    let sv: Vec<u8> = vit_s.iter().map(|c| if *c == '1' { b'1' } else { b'0' }).collect();
    let mut best_off = 0usize;
    let mut best_rate = 0.0f64;
    for off in 0..600usize {
        if rv.len() < off + 500 { break; }
        if sv.len() < off + 500 { break; }
        let mut same = 0usize;
        for i in 0..500 {
            if rv[rv.len() - 500 + i] == sv[sv.len() - off - 500 + i] { same += 1; }
        }
        let r = same as f64 / 500.0;
        if r > best_rate { best_rate = r; best_off = off; }
    }
    println!("末尾整合: 最良 offset={} 一致率 {:.2}%", best_off, best_rate * 100.0);
    // 直接の末尾比較
    let m = rv.len().min(sv.len());
    let mut same = 0usize;
    for i in 0..m {
        if rv[rv.len() - m + i] == sv[sv.len() - m + i] { same += 1; }
    }
    println!("末尾直接比較: {}/{} = {:.2}%", same, m,
        if m > 0 { same as f64 / m as f64 * 100.0 } else { 0.0 });

    // --- 3. **保存済み vit** -> byte 整列 -> RS 入力バイトと一致するか ---
    let use_saved = std::env::var("USE_SAVED").is_ok();
    // deinterleaver の位相。稼働中の復調器と同じにする。
    // push() は `idx % BI_I` で分岐を選ぶため、idx=0 から始めると
    // 別分支になり出力列が一致しない（先頭 1 バイトだけ 0x47 で
    // 2 バイト目以降が全滅する — 実測）。
    let didx: usize = a.get(3)
        .and_then(|x| std::fs::read_to_string(x).ok())
        .and_then(|x| x.trim().parse().ok())
        .unwrap_or(0);
    if didx > 0 { eprintln!("[replay] deinterleaver idx を {} に合わせる", didx); }
    let saved_bits: Vec<u8> = sv.iter().map(|c| if *c == b'1' {1u8} else {0u8}).collect();
    let mut bits: Vec<u8> = if use_saved { saved_bits.clone() } else { replayed.clone() };
    if use_saved { eprintln!("[replay] 保存済み vit を使用 ({} bit)", bits.len()); }
    while bits.len() % 8 != 0 { bits.pop(); }
    let mut bytes: Vec<u8> = Vec::new();
    for chunk in bits.chunks(8) {
        let mut v = 0u8;
        for b in chunk { v = (v << 1) | b; }
        bytes.push(v);
    }
    println!("repack -> {} バイト", bytes.len());

    // byte deinterleave (commutator は 0..11 全部試す)
    let mut best: Option<(f32, usize, Vec<u8>)> = None;
    for c in 0..BI_I.min(bytes.len()) {
        let mut di = ByteDeinterleaver::new();
        // 稼働中の復調器と**同一の分岐位相**に合わせる。
        // `push()` は `idx % BI_I` で分岐を選ぶため、idx=0 から始めると
        // 別の分岐にバイトが乗り出力列が一致しない（実測: 先頭 1 バイト
        // だけ 0x47、2 バイト目以降が全滅）。
        if didx > 0 {
            di.prime_phase(didx);
        }
        let mut stream: Vec<u8> = Vec::new();
        for (j, b) in bytes[c..].iter().enumerate() {
            let o = di.push(*b);
            if j >= BYTE_LATENCY { stream.push(o); }
        }
        // 出力の**全範囲**で RS 入力と一致する窓を探す。
        // 原子スナップショットの vit は RS バイト0の出所から始まるので、
        // deinterleaver の 2244 バイト分のラッシュを消費した
        // 2244 付近に RS バイトが現れるはず。
        let mut win: Option<(f64, usize, Vec<u8>)> = None;
        // `stream` は既に BYTE_LATENCY 分を消費済みなので、
        // 残る 204 バイトが否定できるかどうかだけを判定する。
        if stream.len() >= rs::N {
            for off in 0..=(stream.len() - rs::N) {
                let mut same = 0usize;
                for k in 0..rs::N {
                    if stream[off + k] == bytes[k] { same += 1; }
                }
                let r = same as f64 / rs::N as f64;
                if win.is_none() || r > win.as_ref().unwrap().0 {
                    win = Some((r, off, stream[off..off + rs::N].to_vec()));
                }
            }
            if let Some((r, off, w)) = &win {
                println!("  全窓走査: 最良 offset={} 一致率 {:.2}%", off, r * 100.0);
                let n = w.len().min(8);
                println!("    窓の先頭8: {:02X?}", &w[..n]);
            }
        }
        // 先頭から 204 バイト取り出して target と比較
        let n = rs::N.min(stream.len()).min(target.len());
        let hit = stream[..n].iter().zip(target[..n].iter()).filter(|(a, b)| a == b).count();
        let f = hit as f32 / n.max(1) as f32;
        if best.as_ref().map(|b| f > b.0).unwrap_or(true) {
            best = Some((f, c, stream[..n].to_vec()));
        }
    }
    if let Some((f, c, st)) = best {
        println!("byte 整列 commutator={} 一致率 {:.2}%", c, f * 100.0);
        println!("  再構成先頭8: {:02X?}", &st[..8.min(st.len())]);
        println!("  RS 入力先頭8: {:02X?}", &target[..8.min(target.len())]);
    }
}
