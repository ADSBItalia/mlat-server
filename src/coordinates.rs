pub const SPEED_OF_LIGHT: f64 = 299_792_458.0; // m/s (constants.Cair / c)

pub const WGS84_A: f64 = 6378137.0; // semi-major axis (meters)
pub const WGS84_F: f64 = 1.0 / 298.257223563; // flattening
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F); // semi-minor axis
pub const WGS84_ECC_SQ: f64 = 1.0 - WGS84_B * WGS84_B / (WGS84_A * WGS84_A);

const WGS84_EP: f64 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);
const WGS84_EP2_B: f64 = WGS84_EP * WGS84_B;
const WGS84_E2_A: f64 = WGS84_ECC_SQ * WGS84_A;

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GeodeticPoint {
    pub lat: f64, // degrees (-90 to +90)
    pub lon: f64, // degrees (-180 to +180)
    pub alt: f64, // meters above WGS84 ellipsoid
}

impl GeodeticPoint {
    pub fn new(lat: f64, lon: f64, alt: f64) -> Self {
        Self { lat, lon, alt }
    }

    pub fn to_ecef(&self) -> EcefPoint {
        llh2ecef(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EcefPoint {
    pub x: f64, // meters
    pub y: f64, // meters
    pub z: f64, // meters
}

impl EcefPoint {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn to_geodetic(&self) -> GeodeticPoint {
        ecef2llh(self)
    }

    pub fn distance_to(&self, other: &EcefPoint) -> f64 {
        ecef_distance(self, other)
    }
}

pub fn llh2ecef(geo: &GeodeticPoint) -> EcefPoint {
    let lat_rad = geo.lat.to_radians();
    let lon_rad = geo.lon.to_radians();

    let slat = lat_rad.sin();
    let slng = lon_rad.sin();
    let clat = lat_rad.cos();
    let clng = lon_rad.cos();

    let d = (1.0 - (slat * slat * WGS84_ECC_SQ)).sqrt();
    let rn = WGS84_A / d;

    let x = (rn + geo.alt) * clat * clng;
    let y = (rn + geo.alt) * clat * slng;
    let z = (rn * (1.0 - WGS84_ECC_SQ) + geo.alt) * slat;

    EcefPoint { x, y, z }
}

pub fn ecef2llh(ecef: &EcefPoint) -> GeodeticPoint {
    let lon = ecef.y.atan2(ecef.x);
    let p = (ecef.x * ecef.x + ecef.y * ecef.y).sqrt();

    let th = (WGS84_A * ecef.z).atan2(WGS84_B * p);
    let lat = (ecef.z + WGS84_EP2_B * th.sin().powi(3))
        .atan2(p - WGS84_E2_A * th.cos().powi(3));

    let n = WGS84_A / (1.0 - WGS84_ECC_SQ * lat.sin().powi(2)).sqrt();
    let alt = p / lat.cos() - n;

    GeodeticPoint {
        lat: lat.to_degrees(),
        lon: lon.to_degrees(),
        alt,
    }
}

pub fn ecef_distance(p0: &EcefPoint, p1: &EcefPoint) -> f64 {
    let dx = p0.x - p1.x;
    let dy = p0.y - p1.y;
    let dz = p0.z - p1.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn ecef_vel_to_track_speed(
    geo: &GeodeticPoint,
    vel_ecef: (f64, f64, f64),
) -> (f32, f32, f32) {
    let lat_rad = geo.lat.to_radians();
    let lon_rad = geo.lon.to_radians();
    let (vx, vy, vz) = vel_ecef;

    let slat = lat_rad.sin();
    let clat = lat_rad.cos();
    let slon = lon_rad.sin();
    let clon = lon_rad.cos();

    let v_east = -slon * vx + clon * vy;
    let v_north = -slat * clon * vx - slat * slon * vy + clat * vz;
    let v_up = clat * clon * vx + clat * slon * vy + slat * vz;

    let mut heading = v_east.atan2(v_north).to_degrees();
    if heading < 0.0 {
        heading += 360.0;
    }

    let speed_mps = (v_east * v_east + v_north * v_north).sqrt();
    let speed_kts = speed_mps * 1.943844;
    let vrate_fpm = v_up * 196.8504; // 1 m/s = 196.8504 ft/min

    (heading as f32, speed_kts as f32, vrate_fpm as f32)
}
