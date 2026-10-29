//! JavaScript number semantics, so a kernel matches its TypeScript reference
//! bit for bit. Rust's `f64::max`/`min` ignore NaN and may return either zero,
//! `signum` maps ±0 to ±1, and the platform `exp` is musl's rather than V8's
//! fdlibm. These follow ECMAScript and V8 instead.

// fdlibm's constants are kept digit for digit.
#![allow(clippy::approx_constant, clippy::excessive_precision)]

/// `Math.max(a, b)`: NaN if either is NaN; +0 above -0.
#[inline]
pub fn max(a: f64, b: f64) -> f64 {
    if a > b {
        a
    } else if b > a {
        b
    } else if a == b {
        if a == 0.0 && a.is_sign_negative() { b } else { a }
    } else {
        f64::NAN
    }
}

/// `Math.min(a, b)`: NaN if either is NaN; -0 below +0.
#[inline]
pub fn min(a: f64, b: f64) -> f64 {
    if a < b {
        a
    } else if b < a {
        b
    } else if a == b {
        if a == 0.0 && a.is_sign_positive() { b } else { a }
    } else {
        f64::NAN
    }
}

/// `Math.sign(x)`: ±0 and NaN come back unchanged.
#[inline]
pub fn sign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        x
    }
}

/// `Number.isFinite(x) ? x : fallback`.
#[inline]
pub fn finite_or(x: f64, fallback: f64) -> f64 {
    if x.is_finite() { x } else { fallback }
}

/// `Number.isSafeInteger(x)`.
#[inline]
pub fn is_safe_integer(x: f64) -> bool {
    x.is_finite() && x.trunc() == x && x.abs() <= 9_007_199_254_740_991.0
}

/// `Math.hypot(...values)` as V8 computes it: each magnitude scaled by the
/// largest, the squares Kahan-summed.
pub fn hypot(values: &[f64]) -> f64 {
    let mut max = 0.0_f64;
    let mut nan = false;
    for &value in values {
        if value.is_nan() {
            nan = true;
        } else if value.abs() > max {
            max = value.abs();
        }
    }
    if max == f64::INFINITY {
        return f64::INFINITY;
    }
    if nan {
        return f64::NAN;
    }
    if max == 0.0 {
        return 0.0;
    }
    let mut sum = 0.0_f64;
    let mut compensation = 0.0_f64;
    for &value in values {
        let n = value.abs() / max;
        let summand = n * n - compensation;
        let preliminary = sum + summand;
        compensation = (preliminary - sum) - summand;
        sum = preliminary;
    }
    sum.sqrt() * max
}

/// `Math.exp(x)` as V8 computes it: `base::ieee754::exp`, FreeBSD's fdlibm
/// `e_exp.c` with V8's exact `exp(1) == E`, as compiled for arm64. Clang
/// contracts each `a * b + c` there into a fused multiply-add, so the pinned
/// Node 26 on arm64 differs from plain fdlibm by 1 ulp on about 3 in 10⁴
/// arguments; the fused forms below reproduce it bit for bit. A build without
/// FMA contraction (baseline x86-64) returns plain fdlibm instead.
pub fn exp(x: f64) -> f64 {
    const HALF: [f64; 2] = [0.5, -0.5];
    const O_THRESHOLD: f64 = 7.09782712893383973096e+02;
    const U_THRESHOLD: f64 = -7.45133219101941108420e+02;
    const LN2_HI: [f64; 2] = [6.93147180369123816490e-01, -6.93147180369123816490e-01];
    const LN2_LO: [f64; 2] = [1.90821492927058770002e-10, -1.90821492927058770002e-10];
    const INV_LN2: f64 = 1.44269504088896338700e+00;
    const P1: f64 = 1.66666666666666019037e-01;
    const P2: f64 = -2.77777777770155933842e-03;
    const P3: f64 = 6.61375632143793436117e-05;
    const P4: f64 = -1.65339022054652515390e-06;
    const P5: f64 = 4.13813679705723846039e-08;
    const E: f64 = 2.718281828459045;
    const HUGE: f64 = 1.0e300;
    const TWO_M1000: f64 = 9.33263618503218878990e-302;
    const TWO_1023: f64 = 8.988465674311579539e307;

    let bits = x.to_bits();
    let high = (bits >> 32) as u32;
    let sign = (high >> 31) as usize;
    let hx = high & 0x7fff_ffff;
    if hx >= 0x4086_2e42 {
        if hx >= 0x7ff0_0000 {
            if ((hx & 0x000f_ffff) | bits as u32) != 0 {
                return x + x;
            }
            return if sign == 0 { x } else { 0.0 };
        }
        if x > O_THRESHOLD {
            return HUGE * HUGE;
        }
        if x < U_THRESHOLD {
            return TWO_M1000 * TWO_M1000;
        }
    }

    let mut x = x;
    let mut k: i32 = 0;
    let mut hi = 0.0;
    let mut lo = 0.0;
    if hx > 0x3fd6_2e42 {
        if hx < 0x3ff0_a2b2 {
            if x == 1.0 {
                return E;
            }
            hi = x - LN2_HI[sign];
            lo = LN2_LO[sign];
            k = 1 - 2 * sign as i32;
        } else {
            k = INV_LN2.mul_add(x, HALF[sign]) as i32;
            let t = k as f64;
            // Exact product, so fusing it would change nothing.
            hi = x - t * LN2_HI[0];
            lo = t * LN2_LO[0];
        }
        x = hi - lo;
    } else if hx < 0x3e30_0000 && HUGE + x > 1.0 {
        return 1.0 + x;
    }

    let t = x * x;
    let exponent = if k >= -1021 { k } else { k + 1000 };
    let twopk = f64::from_bits(((0x3ff0_0000_i64 + ((exponent as i64) << 20)) as u64) << 32);
    // x - t * (P1 + t * (P2 + t * (P3 + t * (P4 + t * P5)))), fused.
    let p = t.mul_add(t.mul_add(t.mul_add(t.mul_add(P5, P4), P3), P2), P1);
    let c = (-t).mul_add(p, x);
    if k == 0 {
        return 1.0 - ((x * c) / (c - 2.0) - x);
    }
    let y = 1.0 - ((lo - (x * c) / (2.0 - c)) - hi);
    if k >= -1021 {
        if k == 1024 {
            return y * 2.0 * TWO_1023;
        }
        y * twopk
    } else {
        y * twopk * TWO_M1000
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn min_max_follow_javascript() {
        assert!(max(f64::NAN, 1.0).is_nan() && max(1.0, f64::NAN).is_nan());
        assert!(min(f64::NAN, 1.0).is_nan() && min(1.0, f64::NAN).is_nan());
        assert!(max(-0.0, 0.0).is_sign_positive() && max(0.0, -0.0).is_sign_positive());
        assert!(max(-0.0, -0.0).is_sign_negative());
        assert!(min(0.0, -0.0).is_sign_negative() && min(-0.0, 0.0).is_sign_negative());
        assert!(min(0.0, 0.0).is_sign_positive());
        assert_eq!(max(2.0, 3.0), 3.0);
        assert_eq!(min(2.0, 3.0), 2.0);
    }

    #[test]
    fn sign_keeps_zeros_and_nan() {
        assert!(sign(-0.0).is_sign_negative() && sign(-0.0) == 0.0);
        assert!(sign(0.0).is_sign_positive() && sign(0.0) == 0.0);
        assert!(sign(f64::NAN).is_nan());
        assert_eq!(sign(-3.0), -1.0);
        assert_eq!(sign(f64::INFINITY), 1.0);
    }

    #[test]
    fn exp_edges() {
        assert_eq!(exp(0.0), 1.0);
        assert_eq!(exp(-0.0), 1.0);
        assert_eq!(exp(1.0), 2.718281828459045);
        assert_eq!(exp(f64::INFINITY), f64::INFINITY);
        assert_eq!(exp(f64::NEG_INFINITY), 0.0);
        assert!(exp(f64::NAN).is_nan());
        assert_eq!(exp(710.0), f64::INFINITY);
        assert_eq!(exp(-746.0), 0.0);
    }

    #[test]
    fn hypot_edges() {
        assert_eq!(hypot(&[]), 0.0);
        assert_eq!(hypot(&[3.0, 4.0]), 5.0);
        assert_eq!(hypot(&[f64::NAN, f64::INFINITY]), f64::INFINITY);
        assert!(hypot(&[f64::NAN, 1.0]).is_nan());
        assert_eq!(hypot(&[-0.0, 0.0]), 0.0);
    }
}
