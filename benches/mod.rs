#![feature(test)]
extern crate test;
extern crate velodyne;

use velodyne::{TurnIterator, FullPoint};
use velodyne::packet::PcapSource;

use test::Bencher;

#[bench]
fn bench_hdl64_turn(b: &mut Bencher) {
    let path = "data/hdl64.pcap";
    let source = PcapSource::new(path, false, true).unwrap();
    let mut turn_iter = TurnIterator::hdl64_init(source).unwrap();

    b.iter(|| {
        let res = turn_iter.next().unwrap();
        let point: Vec<FullPoint> = res.unwrap().1;
        test::black_box(point);
    });
}

#[bench]
fn bench_hdl32_turn(b: &mut Bencher) {
    let path = "data/hdl32.pcap";
    let source = PcapSource::new(path, false, true).unwrap();
    let mut turn_iter = TurnIterator::hdl32_init(source);

    b.iter(|| {
        let res = turn_iter.next().unwrap();
        let point: Vec<FullPoint> = res.unwrap().1;
        test::black_box(point);
    });
}
