use crate::{trim_f64, Angle, Distance, GPSBounds, LonLat, EPSILON_DIST};
use ordered_float::NotNan;
use serde_derive::{Deserialize, Serialize};
use std::f64;
use std::fmt;

// This represents world-space in meters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pt2D {
    inner_x: f64,
    inner_y: f64,
}

impl Pt2D {
    pub fn new(x: f64, y: f64) -> Pt2D {
        if !x.is_finite() || !y.is_finite() {
            panic!("Bad Pt2D {}, {}", x, y);
        }

        // TODO enforce >=0

        Pt2D {
            inner_x: trim_f64(x),
            inner_y: trim_f64(y),
        }
    }

    // TODO This is a small first step...
    pub fn approx_eq(self, other: Pt2D, threshold: Distance) -> bool {
        self.dist_to(other) <= threshold
    }

    // Useful shortcut that's easy to refactor in the future.
    pub fn epsilon_eq(self, other: Pt2D) -> bool {
        self.approx_eq(other, EPSILON_DIST)
    }

    pub fn from_gps(gps: LonLat, b: &GPSBounds) -> Option<Pt2D> {
        // TODO hack to construct test maps more easily
        if b.represents_world_space {
            return Some(Pt2D::new(gps.longitude, gps.latitude));
        }

        if !b.contains(gps) {
            return None;
        }

        Some(Pt2D::forcibly_from_gps(gps, b))
    }

    // Can go out of bounds.
    pub fn forcibly_from_gps(gps: LonLat, b: &GPSBounds) -> Pt2D {
        let (width, height) = {
            let pt = b.get_max_world_pt();
            (pt.x(), pt.y())
        };

        let x = (gps.longitude - b.min_lon) / (b.max_lon - b.min_lon) * width;
        // Invert y, so that the northernmost latitude is 0. Screen drawing order, not Cartesian grid.
        let y = height - ((gps.latitude - b.min_lat) / (b.max_lat - b.min_lat) * height);
        Pt2D::new(x, y)
    }

    pub fn to_gps(self, b: &GPSBounds) -> Option<LonLat> {
        if b.represents_world_space {
            let pt = LonLat::new(self.x(), self.y());
            if b.contains(pt) {
                return Some(pt);
            } else {
                return None;
            }
        }

        let (width, height) = {
            let pt = b.get_max_world_pt();
            (pt.x(), pt.y())
        };
        if self.x() < 0.0 || self.y() < 0.0 || self.x() > width || self.y() > height {
            return None;
        }

        let lon = (self.x() / width * (b.max_lon - b.min_lon)) + b.min_lon;
        let lat = b.min_lat + ((b.max_lat - b.min_lat) * (height - self.y()) / height);

        Some(LonLat::new(lon, lat))
    }

    pub fn x(self) -> f64 {
        self.inner_x
    }

    pub fn y(self) -> f64 {
        self.inner_y
    }

    // TODO better name
    pub fn project_away(self, dist: Distance, theta: Angle) -> Pt2D {
        // If negative, caller should use theta.opposite()
        assert!(dist >= Distance::ZERO);

        let (sin, cos) = theta.normalized_radians().sin_cos();
        Pt2D::new(
            self.x() + dist.inner_meters() * cos,
            self.y() + dist.inner_meters() * sin,
        )
    }

    // TODO valid to do euclidean distance on world-space points that're formed from
    // Haversine?
    pub(crate) fn raw_dist_to(self, to: Pt2D) -> f64 {
        ((self.x() - to.x()).powi(2) + (self.y() - to.y()).powi(2)).sqrt()
    }

    pub fn dist_to(self, to: Pt2D) -> Distance {
        Distance::meters(self.raw_dist_to(to))
    }

    pub fn angle_to(self, to: Pt2D) -> Angle {
        // DON'T invert y here
        Angle::new((to.y() - self.y()).atan2(to.x() - self.x()))
    }

    pub fn offset(self, dx: Distance, dy: Distance) -> Pt2D {
        Pt2D::new(self.x() + dx.inner_meters(), self.y() + dy.inner_meters())
    }

    pub fn center(pts: &Vec<Pt2D>) -> Pt2D {
        if pts.is_empty() {
            panic!("Can't find center of 0 points");
        }
        let mut x = 0.0;
        let mut y = 0.0;
        for pt in pts {
            x += pt.x();
            y += pt.y();
        }
        let len = pts.len() as f64;
        Pt2D::new(x / len, y / len)
    }

    // Temporary until Pt2D has proper resolution.
    pub fn approx_dedupe(pts: Vec<Pt2D>, threshold: Distance) -> Vec<Pt2D> {
        let mut result: Vec<Pt2D> = Vec::new();
        for pt in pts {
            if result.is_empty() || !result.last().unwrap().approx_eq(pt, threshold) {
                result.push(pt);
            }
        }
        result
    }

    pub fn find_pts_between(
        pts: &Vec<Pt2D>,
        start: Pt2D,
        end: Pt2D,
        threshold: Distance,
    ) -> Option<Vec<Pt2D>> {
        let mut result = Vec::new();
        for pt in pts {
            if result.is_empty() && pt.approx_eq(start, threshold) {
                result.push(*pt);
            } else if !result.is_empty() {
                result.push(*pt);
            }
            // start and end might be the same.
            if !result.is_empty() && pt.approx_eq(end, threshold) {
                return Some(result);
            }
        }

        // start wasn't in the list!
        if result.is_empty() {
            return None;
        }

        // Go through again, looking for end
        for pt in pts {
            result.push(*pt);
            if pt.approx_eq(end, threshold) {
                return Some(result);
            }
        }
        // Didn't find end
        None
    }

    pub fn to_hashable(self) -> HashablePt2D {
        HashablePt2D::new(self.x(), self.y())
    }
}

impl fmt::Display for Pt2D {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Pt2D({0}, {1})", self.x(), self.y())
    }
}

impl From<HashablePt2D> for Pt2D {
    fn from(pt: HashablePt2D) -> Self {
        Pt2D::new(pt.x(), pt.y())
    }
}

// This isn't opinionated about what the (x, y) represents -- could be lat/lon or world space.
// TODO So rename it HashablePair or something
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, PartialOrd, Ord)]
pub struct HashablePt2D {
    x_nan: NotNan<f64>,
    y_nan: NotNan<f64>,
}

impl HashablePt2D {
    pub fn new(x: f64, y: f64) -> HashablePt2D {
        HashablePt2D {
            x_nan: NotNan::new(x).unwrap(),
            y_nan: NotNan::new(y).unwrap(),
        }
    }

    pub fn x(&self) -> f64 {
        self.x_nan.into_inner()
    }

    pub fn y(&self) -> f64 {
        self.y_nan.into_inner()
    }
}

impl From<Pt2D> for HashablePt2D {
    fn from(pt: Pt2D) -> Self {
        HashablePt2D::new(pt.x(), pt.y())
    }
}
