use crate::util::math::Vec3d;

/// - `base_cart`: pozycja punktu bazowego (środek Ziemi + promień + wysokość bazowa) w układzie geocentrycznym.
/// - `east`: przesunięcie na wschód (km)
/// - `north`: przesunięcie na północ (km)
/// - `up`: wysokość nad powierzchnią (km) (dodatnia w górę)
pub fn enu_to_cartesian_offset(base_cart: Vec3d, east: f64, north: f64, up: f64) -> Vec3d {
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
