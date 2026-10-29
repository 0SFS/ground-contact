//! Geodesy and aircraft frames, ported from `foss-earth/cameraMath`
//! (`geodeticToEcef`, `ecefToGeodetic`) and `src/collisionGeometry.ts`
//! (`groundContactBodyPosition`, `bodyToLocalRotation`, `bodyOffsetLocal`,
//! `metresPerDegree`). The arithmetic follows the references; `sin`, `cos`
//! and `atan2` are libm's, which can differ from V8's by an ulp.

use core::f64::consts::PI;

pub const WGS84_A: f64 = 6_378_137.0;
pub const WGS84_F: f64 = 1.0 / 298.257223563;
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);
pub const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F;
pub const WGS84_EP2: f64 = WGS84_E2 / (1.0 - WGS84_E2);
/// `collisionGeometry.ts` rounds the eccentricity for `metresPerDegree`.
pub const FOOTPRINT_E2: f64 = 6.69437999014e-3;
pub const DEG_TO_RAD: f64 = PI / 180.0;
pub const INCH_METERS: f64 = 0.0254;
pub const FOOT_METERS: f64 = 0.3048;

pub type V3 = [f64; 3];

#[inline]
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
/// Row-major 3×3 times a vector.
#[inline]
pub fn mul(m: &[f64; 9], v: V3) -> V3 {
    [
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    ]
}
#[inline]
pub fn all_finite(values: &[f64]) -> bool {
    values.iter().all(|v| v.is_finite())
}

/// `geodeticToEcef(latRad, lonRad, altMeters)`.
pub fn geodetic_to_ecef(lat_rad: f64, lon_rad: f64, alt_m: f64) -> V3 {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    [
        (n + alt_m) * cos_lat * cos_lon,
        (n + alt_m) * cos_lat * sin_lon,
        (n * (1.0 - WGS84_E2) + alt_m) * sin_lat,
    ]
}

/// `ecefToGeodetic(x, y, z)`: Bowring's closed form, `[latRad, lonRad, altMeters]`.
pub fn ecef_to_geodetic(p: V3) -> V3 {
    let [x, y, z] = p;
    let r = (x * x + y * y).sqrt();
    let lon_rad = y.atan2(x);
    let theta = (z * WGS84_A).atan2(r * WGS84_B);
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let lat_rad = (z + WGS84_EP2 * WGS84_B * sin_theta * sin_theta * sin_theta)
        .atan2(r - WGS84_E2 * WGS84_A * cos_theta * cos_theta * cos_theta);
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let alt_m = if cos_lat.abs() > 1e-10 {
        r / cos_lat - n
    } else {
        z.abs() / sin_lat.abs() - n * (1.0 - WGS84_E2)
    };
    [lat_rad, lon_rad, alt_m]
}



/// `bodyToLocalRotation`: row-major, `(north, east, down) = M (forward, right, down)`.
pub fn body_to_local_rotation(roll: f64, pitch: f64, heading: f64) -> [f64; 9] {
    let (sr, cr) = (roll.sin(), roll.cos());
    let (sp, cp) = (pitch.sin(), pitch.cos());
    let (sh, ch) = (heading.sin(), heading.cos());
    [
        cp * ch,
        sr * sp * ch - cr * sh,
        cr * sp * ch + sr * sh,
        cp * sh,
        sr * sp * sh + cr * ch,
        cr * sp * sh - sr * ch,
        -sp,
        sr * cp,
        cr * cp,
    ]
}

/// `groundContactBodyPosition`: structural inches (aft, right, up) relative to
/// the CG as metres `[left, up, forward]`.
#[inline]
pub fn contact_body_position(contact: V3, cg: V3) -> V3 {
    [
        (cg[1] - contact[1]) * INCH_METERS,
        (contact[2] - cg[2]) * INCH_METERS,
        (cg[0] - contact[0]) * INCH_METERS,
    ]
}

/// `bodyOffsetLocal`: a `[left, up, forward]` offset as `[north, east, down]`.
#[inline]
pub fn body_offset_local(offset: V3, rotation: &[f64; 9]) -> V3 {
    let forward = offset[2];
    let right = -offset[0];
    let down = -offset[1];
    mul(rotation, [forward, right, down])
}

/// `metresPerDegree(latDeg)`: `[lat, lon]` metres per degree.
pub fn metres_per_degree(lat_deg: f64) -> [f64; 2] {
    let lat = lat_deg * PI / 180.0;
    let sine = lat.sin();
    let w = 1.0 - FOOTPRINT_E2 * sine * sine;
    let meridian = WGS84_A * (1.0 - FOOTPRINT_E2) / (w * w.sqrt());
    let normal = WGS84_A / w.sqrt();
    [meridian * PI / 180.0, normal * lat.cos() * PI / 180.0]
}

/// `((lon + 540) % 360) - 180`, as `terrainContact.ts` wraps a footprint.
#[inline]
pub fn wrap_lon(lon_deg: f64) -> f64 {
    ((lon_deg + 540.0) % 360.0) - 180.0
}



#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geodetic_round_trip() {
        for &(lat, lon, alt) in &[(44.88, -93.22, 256.0), (-33.9, 151.2, 3.0), (89.9, 10.0, 1000.0), (0.0, 179.99, -20.0)] {
            let p = geodetic_to_ecef(lat * DEG_TO_RAD, lon * DEG_TO_RAD, alt);
            let [la, lo, al] = ecef_to_geodetic(p);
            assert!((la / DEG_TO_RAD - lat).abs() < 1e-9, "{lat}");
            assert!((lo / DEG_TO_RAD - lon).abs() < 1e-9, "{lon}");
            assert!((al - alt).abs() < 2e-3, "{alt}: {al}");
        }
    }

    #[test]
    fn body_to_local_is_a_rotation() {
        let m = body_to_local_rotation(0.3, -0.2, 2.5);
        let column = |c: usize| [m[c], m[3 + c], m[6 + c]];
        let dot = |a: V3, b: V3| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        for i in 0..3 {
            assert!((dot(column(i), column(i)) - 1.0).abs() < 1e-15);
            for j in 0..i {
                assert!(dot(column(i), column(j)).abs() < 1e-15);
            }
        }
        // Right-handed: forward × right = down.
        let c = cross(column(0), column(1));
        assert!((0..3).all(|a| (c[a] - column(2)[a]).abs() < 1e-15));
    }

    #[test]
    fn longitudes_wrap() {
        assert_eq!(wrap_lon(179.5 + 1.0), -179.5);
        assert_eq!(wrap_lon(-180.5), 179.5);
    }
}
