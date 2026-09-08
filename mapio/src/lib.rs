use anyhow::{anyhow, Result};
use serde::Deserialize;
use std::path::Path;

#[derive(Debug, Deserialize)]
pub struct MapYaml {
    pub image: String,
    pub resolution: f32,
    pub origin: [f32; 3], // [x, y, yaw]
    pub negate: i32,
    pub occupied_thresh: f32,
    pub free_thresh: f32,
}

/// Static occupancy grid loaded from a ROS 2 map_server-style PGM + YAML pair.
/// Cell values: 0 = free, 1 = occupied, -1 = unknown.
pub struct StaticMap {
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub width: usize,
    pub height: usize,
    pub cells: Vec<i8>,
}

impl StaticMap {
    /// Load a map from a YAML metadata file (expects the PGM to be alongside it,
    /// as specified by the `image:` field).
    pub fn load(yaml_path: &Path) -> Result<Self> {
        let yaml_str = std::fs::read_to_string(yaml_path)?;
        let meta: MapYaml = serde_yaml::from_str(&yaml_str)?;

        let pgm_path = yaml_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(&meta.image);
        let (width, height, raw) = read_pgm(&pgm_path)?;

        // ROS map_server convention: PGM pixel value -> occupancy probability.
        // White (255) = free, Black (0) = occupied, by default (negate=0).
        // Probability p = (255 - pixel) / 255.0 (unless negate flips it).
        // occupied if p > occupied_thresh, free if p < free_thresh, else unknown.
        let mut cells = vec![-1i8; width * height];
        for (i, &pixel) in raw.iter().enumerate() {
            let mut p = (255.0 - pixel as f32) / 255.0;
            if meta.negate != 0 {
                p = 1.0 - p;
            }
            cells[i] = if p > meta.occupied_thresh {
                1
            } else if p < meta.free_thresh {
                0
            } else {
                -1
            };
        }

        // PGM rows go top-to-bottom, but map_server's convention has row 0 = bottom
        // (increasing y). Flip vertically to match world coordinates.
        let mut flipped = vec![-1i8; width * height];
        for y in 0..height {
            let src_row = height - 1 - y;
            flipped[y * width..(y + 1) * width]
                .copy_from_slice(&cells[src_row * width..(src_row + 1) * width]);
        }

        Ok(Self {
            resolution: meta.resolution,
            origin_x: meta.origin[0],
            origin_y: meta.origin[1],
            width,
            height,
            cells: flipped,
        })
    }

    /// Convert world coordinates (meters) to grid cell indices, if in bounds.
    pub fn world_to_cell(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let cx = ((x - self.origin_x) / self.resolution) as isize;
        let cy = ((y - self.origin_y) / self.resolution) as isize;
        if cx < 0 || cy < 0 || cx as usize >= self.width || cy as usize >= self.height {
            return None;
        }
        Some((cx as usize, cy as usize))
    }

    /// Convert grid cell indices to world coordinates (cell center, meters).
    pub fn cell_to_world(&self, cx: usize, cy: usize) -> (f32, f32) {
        let x = self.origin_x + (cx as f32 + 0.5) * self.resolution;
        let y = self.origin_y + (cy as f32 + 0.5) * self.resolution;
        (x, y)
    }

    pub fn is_occupied(&self, cx: usize, cy: usize) -> bool {
        self.cells[cy * self.width + cx] == 1
    }

    pub fn is_free(&self, cx: usize, cy: usize) -> bool {
        self.cells[cy * self.width + cx] == 0
    }

    pub fn occupied_count(&self) -> usize {
        self.cells.iter().filter(|&&c| c == 1).count()
    }

    pub fn free_count(&self) -> usize {
        self.cells.iter().filter(|&&c| c == 0).count()
    }

    pub fn unknown_count(&self) -> usize {
        self.cells.iter().filter(|&&c| c == -1).count()
    }
}

/// Minimal PGM (P5, binary, 8-bit grayscale) reader. Returns (width, height, pixels).
fn read_pgm(path: &Path) -> Result<(usize, usize, Vec<u8>)> {
    let data = std::fs::read(path)?;
    let mut pos = 0;

    let magic = read_token(&data, &mut pos)?;
    if magic != "P5" {
        return Err(anyhow!("Unsupported PGM format: {} (only P5 binary supported)", magic));
    }

    let width: usize = read_token(&data, &mut pos)?.parse()?;
    let height: usize = read_token(&data, &mut pos)?.parse()?;
    let maxval: usize = read_token(&data, &mut pos)?.parse()?;
    if maxval > 255 {
        return Err(anyhow!("Only 8-bit PGM supported (maxval <= 255)"));
    }

    // Single whitespace byte separates header from binary data
    pos += 1;
    let pixel_data = data[pos..pos + width * height].to_vec();

    Ok((width, height, pixel_data))
}

/// Reads the next whitespace-separated token from PGM header, skipping '#' comments.
fn read_token(data: &[u8], pos: &mut usize) -> Result<String> {
    // Skip whitespace and comments
    loop {
        while *pos < data.len() && (data[*pos] as char).is_whitespace() {
            *pos += 1;
        }
        if *pos < data.len() && data[*pos] == b'#' {
            while *pos < data.len() && data[*pos] != b'\n' {
                *pos += 1;
            }
        } else {
            break;
        }
    }
    let start = *pos;
    while *pos < data.len() && !(data[*pos] as char).is_whitespace() {
        *pos += 1;
    }
    if start == *pos {
        return Err(anyhow!("Unexpected end of PGM header"));
    }
    Ok(String::from_utf8_lossy(&data[start..*pos]).to_string())
}
