use anyhow::{bail, Result};
use geo::{prelude::*, Coord, LineString, Polygon, Rect};
use itertools::chain;
use log::warn;
use num::Float;
use sfcgal::{CoordSeq, ToCoordinates, ToSFCGAL};
use std::fmt::Debug;

/// Computes the IoU among a rectangle and a polygon.
pub fn rect_hull_iou<T>(rect: &Rect<T>, hull: &Polygon<T>) -> Result<T>
where
    T: Debug + Float,
{
    let epsilon = T::from(1e-6).unwrap();

    let intersection = match rect_hull_intersection(rect, hull)? {
        Some(intersection) => intersection,
        None => return Ok(T::zero()),
    };
    let rect_area = rect.unsigned_area();
    let hull_area = hull.unsigned_area();
    let intersection_area = intersection.unsigned_area();

    let roi = if rect_area <= epsilon || hull_area <= epsilon {
        T::zero()
    } else {
        intersection_area * (rect_area.recip() + hull_area.recip())
    };

    Ok(roi)
}

/// Computes the intersection polygon among a rectangle and a polygon.
pub fn rect_hull_intersection<T>(rect: &Rect<T>, hull: &Polygon<T>) -> Result<Option<Polygon<T>>>
where
    T: Debug + Float,
{
    let cast_f64 =
        |(x, y): (T, T)| -> (f64, f64) { (num::cast(x).unwrap(), num::cast(y).unwrap()) };
    let cast_t = |(x, y): (f64, f64)| -> (T, T) { (num::cast(x).unwrap(), num::cast(y).unwrap()) };

    let rect = {
        let ll = rect.min();
        let ru = rect.max();
        let lu = Coord { x: ll.x, y: ru.y };
        let rl = Coord { x: ru.x, y: ll.y };
        let exterior: Vec<(f64, f64)> = [ll.x_y(), rl.x_y(), ru.x_y(), lu.x_y(), ll.x_y()]
            .into_iter()
            .map(cast_f64)
            .collect();
        CoordSeq::Polygon(vec![exterior]).to_sfcgal()?
    };
    let hull = {
        let exterior: Vec<_> = hull
            .exterior()
            .points()
            .map(|point| point.x_y())
            .map(cast_f64)
            .collect();
        let interiors = hull.interiors().iter().map(|linestring| {
            let points: Vec<_> = linestring
                .points()
                .map(|point| point.x_y())
                .map(cast_f64)
                .collect();
            points
        });
        let linestrings: Vec<_> = chain!([exterior], interiors).collect();
        let polygon = CoordSeq::Polygon(linestrings).to_sfcgal().ok();

        match polygon {
            Some(polygon) => polygon,
            None => {
                bail!("not a valid polygon");
            }
        }
    };
    let intersection = {
        let intersection = rect.intersection(&hull).ok();
        let intersection = match intersection {
            Some(int) => int,
            None => {
                bail!("failed to compute polygon intersection");
            }
        };
        intersection.to_coordinates::<(f64, f64)>()?
    };
    let polygon = match intersection {
        CoordSeq::Polygon(linestrings) => {
            let mut linestrings_iter = linestrings.into_iter();
            let exterior = LineString(
                linestrings_iter
                    .next()
                    .unwrap()
                    .into_iter()
                    .map(cast_t)
                    .map(|(x, y)| Coord { x, y })
                    .collect(),
            );
            let interiors: Vec<_> = linestrings_iter
                .map(|linestring| {
                    LineString(
                        linestring
                            .into_iter()
                            .map(cast_t)
                            .map(|(x, y)| Coord { x, y })
                            .collect(),
                    )
                })
                .collect();
            Polygon::new(exterior, interiors)
        }
        CoordSeq::Triangle(linestring) => {
            let exterior = LineString(
                linestring
                    .into_iter()
                    .map(cast_t)
                    .map(|(x, y)| Coord { x, y })
                    .collect(),
            );
            Polygon::new(exterior, vec![])
        }
        CoordSeq::Geometrycollection(collection) => {
            assert!(collection.is_empty());
            return Ok(None);
        }
        _ => {
            warn!("unexpected intersection");
            return Ok(None);
        }
    };

    Ok(Some(polygon))
}
