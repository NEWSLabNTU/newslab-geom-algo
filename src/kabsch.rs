use crate::centroid_of_points;
use nalgebra::{
    Const, Isometry3, Matrix3, Matrix3xX, Point3, RealField, Scalar, Translation3, UnitQuaternion,
    Vector3, SVD,
};
use num::Float;
use std::borrow::Borrow;

unzip_n::unzip_n!(2);
unzip_n::unzip_n!(4);

/// Computes the optimal rotation from a point set to target point
/// set.
///
/// The function accepts an iterator of point pairs `(P, Q)`, where
/// `Q` is target point for `P`.
pub fn kabsch<T, P, Q, I>(input_target_pairs: I) -> Option<Isometry3<T>>
where
    T: Scalar + Float + Default + RealField,
    P: Borrow<Point3<T>>,
    Q: Borrow<Point3<T>>,
    I: IntoIterator<Item = (P, Q)>,
{
    let (input_points, target_points) = input_target_pairs.into_iter().unzip_n_vec();

    // compute centroids
    let input_centroid = centroid_of_points(input_points.iter().map(Borrow::borrow))?;
    let target_centroid = centroid_of_points(target_points.iter().map(Borrow::borrow))?;

    // translate centroid to origin
    let input_points: Vec<_> = input_points
        .into_iter()
        .map(|point| point.borrow() - input_centroid)
        .collect();
    let target_points: Vec<_> = target_points
        .into_iter()
        .map(|point| point.borrow() - target_centroid)
        .collect();

    let input_matrix = Matrix3xX::from_columns(&input_points);
    let target_matrix = Matrix3xX::from_columns(&target_points);
    let covariance = input_matrix * target_matrix.transpose();

    let svd = SVD::new(covariance, true, true);
    let svd_u = svd.u.unwrap();
    let svd_v_t = svd.v_t.unwrap();

    let det = (svd_u * svd_v_t).determinant();
    let d_sign = Float::signum(det);
    let one = T::one();
    let rotation_t = svd_u * Matrix3::from_diagonal(&Vector3::new(one, one, d_sign)) * svd_v_t;

    // it transposes the matrix and converts from DMatrix to Matrix3 type.
    // `rotation_t.as_slice()` iterates elements in column-major order,
    // and construct matrix by `from_row_slice_generic()` in row-major order.
    let rotation = Matrix3::from_row_slice_generic(Const::<3>, Const::<3>, rotation_t.as_slice());

    let quaternion = UnitQuaternion::from_matrix(&rotation);

    let translation: Translation3<T> =
        (target_centroid.coords - quaternion * input_centroid.coords).into();

    let isometry = Isometry3 {
        translation,
        rotation: quaternion,
    };
    Some(isometry)
}

#[cfg(test)]
mod tests {

    use approx::assert_abs_diff_eq;
    use itertools::izip;
    use nalgebra::{Isometry3, Point3, Vector3};

    #[test]
    fn kabsch_test() {
        let num_points = 16;
        let input_points: Vec<Point3<f64>> = (0..num_points)
            .map(|_| Vector3::new_random().into())
            .collect();
        let target_points: Vec<Point3<f64>> = (0..num_points)
            .map(|_| Vector3::new_random().into())
            .collect();

        let transform = super::kabsch(izip!(input_points.clone(), target_points.clone())).unwrap();

        let optimized_input_points = input_points.into_iter().map(|point| transform * point);
        let unit_transform = super::kabsch(izip!(optimized_input_points, target_points)).unwrap();
        assert_abs_diff_eq!(unit_transform, Isometry3::identity(), epsilon = 1e-5);
    }
}
