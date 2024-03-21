use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;

#[derive(Debug)]
pub struct Track {
    pub raw_track: Vec<Vec<f64>>, // Assuming the raw_track is a 2D vector of f64
    pub elements: usize,
}

impl Track {
    pub fn new(filename: &str) -> io::Result<Self> {
        let path = Path::new(filename);
        let file = File::open(&path)?;
        let reader = io::BufReader::new(file);

        let mut raw_track = Vec::new();

        for (index, line) in reader.lines().enumerate() {
            let line = line?;
            if index < 2 { continue; } // Skip the first two lines

            let values: Vec<f64> = line.split(',')
                .map(|s| s.parse().unwrap_or(0.0))
                .collect();

            raw_track.push(values);
        }

        let elements = raw_track.len();

        Ok(Self { raw_track, elements })
    }
}