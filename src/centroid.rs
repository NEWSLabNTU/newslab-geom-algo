use collected::{AddVal, SumVal};
use nalgebra::{Point3, RealField, Scalar};
use num::Float;
use std::borrow::Borrow;

unzip_n::unzip_n!(2);
unzip_n::unzip_n!(4);

/// Computes the centroid point from an iterator of points, and moves
/// the points so that the centroid becomes zero.
pub fn shift_centroid_to_origin<T, I>(points: I) -> Option<impl Iterator<Item = [T; 3]>>
where
    T: Scalar + Float + RealField + Default,
    I: IntoIterator<Item = [T; 3]>,
{
    Some(shift_centroid_to_origin_na(points.into_iter().map(Point3::from))?.map(<[T; 3]>::from))
}

/// Computes the centroid point from an iterator of points, and moves
/// the points so that the centroid becomes zero.
pub fn shift_centroid_to_origin_na<T, P, I>(points: I) -> Option<impl Iterator<Item = Point3<T>>>
where
    T: Scalar + Float + RealField + Default,
    P: Borrow<Point3<T>>,
    I: IntoIterator<Item = P>,
{
    let points: Vec<_> = points.into_iter().collect();
    let centroid = centroid_of_points_na(points.iter().map(Borrow::borrow))?;

    let iter = points
        .into_iter()
        .map(move |point| Point3::from(point.borrow() - centroid));
    Some(iter)
}

/// Computes the centroid from an iterator of points.
pub fn centroid_of_points<T, I>(iter: I) -> Option<[T; 3]>
where
    T: Scalar + Float + Default,
    I: IntoIterator<Item = [T; 3]>,
{
    centroid_of_points_na(iter.into_iter().map(Point3::from)).map(|p| p.into())
}

/// Computes the centroid from an iterator of points.
pub fn centroid_of_points_na<T, P, I>(iter: I) -> Option<Point3<T>>
where
    T: Scalar + Float + Default,
    P: Borrow<Point3<T>>,
    I: IntoIterator<Item = P>,
{
    let (x_sum, y_sum, z_sum, num_points): (AddVal<T>, AddVal<T>, AddVal<T>, SumVal<usize>) = iter
        .into_iter()
        .map(|point| {
            let point = point.borrow();
            (point.x, point.y, point.z, 1)
        })
        .unzip_n();

    let num_points = T::from(num_points.into_inner()).unwrap();
    let zero = T::zero();

    let x_mean = x_sum.into_inner().unwrap_or(zero) / num_points;
    let y_mean = y_sum.into_inner().unwrap_or(zero) / num_points;
    let z_mean = z_sum.into_inner().unwrap_or(zero) / num_points;

    Some(Point3::new(x_mean, y_mean, z_mean))
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn centroid_test() {
        let num_points = 16;
        let points: Vec<Point3<f64>> = (0..num_points)
            .map(|_| Vector3::new_random().into())
            .collect();
        let centroid = super::centroid_of_points_na(points.clone()).unwrap();

        let normalized_points = points
            .into_iter()
            .map(|point| Point3::from(point - centroid));
        let zero = super::centroid_of_points_na(normalized_points).unwrap();
        assert_abs_diff_eq!(zero, Point3::origin());
    }
}
