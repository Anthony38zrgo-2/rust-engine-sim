//! Physical constants and unit helpers (SI base units).

pub const PI: f64 = 3.14159265359;
/// Universal gas constant [J/(mol·K)]
pub const R: f64 = 8.31446261815324;
pub const ROOT_2: f64 = 1.4142135623730951;
pub const E: f64 = 2.718281828459045;

// Force
pub const N: f64 = 1.0;
pub const LBF: f64 = N * 4.44822;

// Mass
pub const KG: f64 = 1.0;
pub const G: f64 = KG / 1000.0;
pub const LB: f64 = 0.45359237 * KG;

// Distance
pub const M: f64 = 1.0;
pub const CM: f64 = M / 100.0;
pub const MM: f64 = M / 1000.0;
pub const INCH: f64 = CM * 2.54;
pub const FOOT: f64 = INCH * 12.0;
pub const THOU: f64 = INCH / 1000.0;

// Time
pub const SEC: f64 = 1.0;

// Torque
pub const NM: f64 = N * M;
pub const FT_LB: f64 = FOOT * LBF;

// Power
pub const W: f64 = NM / SEC;
pub const KW: f64 = W * 1000.0;
pub const HP: f64 = 745.699872 * W;
pub const BHP: f64 = HP;

// Volume
pub const M3: f64 = 1.0;
pub const CC: f64 = CM * CM * CM;
pub const ML: f64 = CC;
pub const L: f64 = ML * 1000.0;

// Molecular
pub const MOL: f64 = 1.0;
pub const KMOL: f64 = MOL / 1000.0;

// Flow-rate (moles)
pub const MOL_PER_SEC: f64 = MOL / SEC;
/// Standard cubic feet per minute, in mol/s (as in original units.h)
pub const SCFM: f64 = 0.002641 * 453.59237 / 60.0;

// Area
pub const M2: f64 = 1.0;
pub const CM2: f64 = CM * CM;

// Pressure
pub const PA: f64 = 1.0;
pub const KPA: f64 = PA * 1000.0;
pub const ATM: f64 = 101.325 * KPA;
pub const MBAR: f64 = PA * 100.0;
pub const BAR: f64 = MBAR * 1000.0;
pub const PSI: f64 = LBF / (INCH * INCH);
pub const IN_HG: f64 = PA * 3386.3886666666713;
pub const IN_H2O: f64 = IN_HG * 0.0734824;

// Temperature
pub const K0: f64 = 273.15;

// Energy
pub const J: f64 = 1.0;
pub const KJ: f64 = J * 1000.0;

// Angles
pub const RAD: f64 = 1.0;
pub const DEG: f64 = RAD * (PI / 180.0);

/// Molar mass of dry air [kg/mol]
pub const AIR_MOLECULAR_MASS: f64 = 0.02897;

/// RPM → rad/s
pub const fn rpm(rpm: f64) -> f64 {
    rpm * 0.104719755
}

/// rad/s → RPM
pub const fn to_rpm(rad_s: f64) -> f64 {
    rad_s / 0.104719755
}

/// Celsius → Kelvin
pub const fn celsius(t_c: f64) -> f64 {
    t_c + K0
}

/// Torque in ft·lb → N·m
pub const fn ft_lb(v: f64) -> f64 {
    v * FT_LB
}

/// Length in mm → m
pub const fn mm(v: f64) -> f64 {
    v * MM
}

/// Length in inches → m
pub const fn inch(v: f64) -> f64 {
    v * INCH
}

/// Volume in cc → m³
pub const fn cc(v: f64) -> f64 {
    v * CC
}

/// Volume in liters → m³
pub const fn liters(v: f64) -> f64 {
    v * L
}

/// Flow rate in scfm → mol/s
pub const fn scfm(v: f64) -> f64 {
    v * SCFM
}

/// Angle in degrees → rad
pub const fn deg(v: f64) -> f64 {
    v * DEG
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rpm_roundtrip() {
        assert!((to_rpm(rpm(6000.0)) - 6000.0).abs() < 1e-9);
    }

    #[test]
    fn one_rev_per_sec_is_60rpm() {
        assert!((to_rpm(2.0 * PI) - 60.0).abs() < 1e-6);
    }

    #[test]
    fn standard_pressure() {
        assert!((ATM - 101325.0).abs() < 1e-6);
    }
}
