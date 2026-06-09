use crate::util::math::Vec3d;

/// - `base_cart`: pozycja punktu bazowego (środek Ziemi + promień + wysokość bazowa) w układzie geocentrycznym.
/// - `east`: przesunięcie na wschód (km)
/// - `north`: przesunięcie na północ (km)
/// - `up`: wysokość nad powierzchnią (km) (dodatnia w górę)
pub fn enu_vector_to_cartesian(base_cart: Vec3d, east: f64, north: f64, up: f64) -> Vec3d {
    let r = base_cart.norm();
    if r < 1e-6 {
        return Vec3d::new(0.0, 0.0, 0.0);
    }
    // Oblicz szerokość i długość geograficzną punktu bazowego (zakładając kulistą Ziemię)
    let lat = (base_cart.y / r).asin();
    let lon = base_cart.z.atan2(base_cart.x);
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();

    // Wersory lokalnego układu ENU w geocentrycznym:
    let east_vec = Vec3d::new(-sin_lon, 0.0, cos_lon);
    let north_vec = Vec3d::new(-sin_lat * cos_lon, cos_lat, -sin_lat * sin_lon);
    let up_vec = Vec3d::new(cos_lat * cos_lon, sin_lat, cos_lat * sin_lon);

    east_vec * east + north_vec * north + up_vec * up
}

/// Zamienia współrzędne geograficzne (stopnie) na wektor kartezjański (km)
/// `radius_km` – odległość od środka planety (np. promień + wysokość)
pub fn geographic_to_cartesian(latitude_deg: f64, longitude_deg: f64, radius_km: f64) -> Vec3d {
    let lat = latitude_deg.to_radians();
    let lon = longitude_deg.to_radians();
    let x = radius_km * lat.cos() * lon.cos();
    let y = radius_km * lat.sin();
    let z = radius_km * lat.cos() * lon.sin();
    Vec3d::new(x, y, z)
}

pub fn earth_rotation_velocity(point_cart: Vec3d) -> Vec3d {
    let omega = 2.0 * std::f64::consts::PI / 86164.0; // rad/s (dzień gwiazdowy ~23h56m4s)
    // v = ω × r
    Vec3d::new(
        -omega * point_cart.y,   // ω = (0,0,omega) → ω × r = ( -ω*rz, 0, ω*rx )? Poprawne:
        // Poprawny iloczyn wektorowy dla ω = (0,0,ω) i r = (x,y,z) daje ( -ω*y, ω*x, 0 )
        // Ale Ziemia obraca się wokół osi Y? W astronomii oś Z to oś obrotu. Ustalmy:
        // ω = (0,0,omega). Wtedy ω × r = ( -omega*y, omega*x, 0 ).
        -omega * point_cart.x,
        0.0,
    )
}
