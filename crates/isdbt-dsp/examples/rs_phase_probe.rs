use isdbt_dsp::deinterleave::{data_carrier_indices, freq_deinterleave, TimeDeinterleaver};
use isdbt_dsp::demap::{qpsk_soft, BitDeinterleaverQpsk};
use isdbt_dsp::demod::OfdmDemod;
use isdbt_dsp::equalize::{equalize, estimate_channel, phase_scores_gr_isdbt, track_symbol_phases};
use isdbt_dsp::iq::u8_iq_to_complex;
use isdbt_dsp::params::{FFT_LEN, GuardInterval};
use isdbt_dsp::pilots::SegmentPilots;
use isdbt_dsp::ts::{best_sync_phase, ByteDeinterleaver, BI_M, BI_I};
use isdbt_dsp::viterbi::{depuncture, Viterbi, PUNCTURE_2_3};
use isdbt_dsp::demap::BitDeinterleaverQpsk as BDI;
use num_complex::Complex32;
use std::{env, fs};
fn main(){let mut a=env::args().skip(1);let p=a.next().unwrap();let _fs:f64=a.next().unwrap().parse().unwrap();let n:usize=a.next().map(|x|x.parse().unwrap()).unwrap_or(12000);let b=fs::read(p).unwrap();let mut s=u8_iq_to_complex(&b);let mean=s.iter().sum::<Complex32>()/s.len() as f32;for v in s.iter_mut(){*v-=mean}let off=(s.len()/10).min(50000);let ss=&s[off..];let est=isdbt_dsp::sync::estimate_symbol_sync(ss,FFT_LEN,GuardInterval::G1_8).unwrap();let d=OfdmDemod::new(FFT_LEN);let sp=d.demod_stream_tracked(ss,est.symbol_start,est.guard,est.cfo_subcarriers,n,8);let pi=SegmentPilots::center_1seg();let rows:Vec<[f32;4]>=sp.iter().map(|x|phase_scores_gr_isdbt(&x[308..740],&pi)).collect();let ph=track_symbol_phases(&rows);let mut tdi=TimeDeinterleaver::new(4);let mut bdi=BDI::new();let mut coded=Vec::new();for (k,x) in sp.iter().enumerate(){let seg=x[308..740].to_vec();let h=estimate_channel(&seg,ph[k],&pi);let e=equalize(&seg,&h);let data:Vec<Complex32>=data_carrier_indices(ph[k],&pi).into_iter().map(|l|e[l]).collect();for v in tdi.push_symbol(&freq_deinterleave(&data)){let z=bdi.push(qpsk_soft(v));coded.push(z[1]);coded.push(z[0]);}}let bits=Viterbi::new().decode(&depuncture(&coded,&PUNCTURE_2_3));let bytes=isdbt_dsp::ts::pack_bits_msb(&bits,0);let lat=BI_M*BI_I*(BI_I-1);for c in 0..BI_I{let mut di=ByteDeinterleaver::new();let mut st=Vec::new();for (j,&x) in bytes[c..].iter().enumerate(){let o=di.push(x);if j>=lat{st.push(o)}}let (bp,score)=best_sync_phase(&st);let mut v:Vec<(usize,f32)>= (0..204).map(|p|(p,isdbt_dsp::ts::sync_score_at(&st,p))).collect();v.sort_by(|a,b|b.1.partial_cmp(&a.1).unwrap());println!("comm={c} best={bp} score={score:.3} top={:?}",&v[..8]);}}
