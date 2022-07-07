use bevy::prelude::*;
use crate::Rot;

pub struct Contact {
    pub penetration: f32,
    pub normal: Vec2,
}

pub fn ball_ball(pos_a: Vec2, radius_a: f32, pos_b: Vec2, radius_b: f32) -> Option<Contact> {
    let ab = pos_b - pos_a;
    let combined_radius = radius_a + radius_b;
    let ab_sqr_len = ab.length_squared();
    if ab_sqr_len < combined_radius * combined_radius {
        let ab_length = ab_sqr_len.sqrt();
        let penetration = combined_radius - ab_length;
        let normal = ab / ab_length;
        Some(Contact {
            normal,
            penetration,
        })
    } else {
        None
    }
}

pub fn ball_box(pos_a: Vec2, radius_a: f32, pos_b: Vec2, size_b: Vec2) -> Option<Contact> {
    let box_to_circle = pos_a - pos_b;
    let box_to_circle_abs = box_to_circle.abs();
    let half_extents = size_b / 2.;
    let corner_to_center = box_to_circle_abs - half_extents;
    let r = radius_a;
    if corner_to_center.x > r || corner_to_center.y > r {
        return None;
    }

    let s = box_to_circle.signum();

    let (n, penetration) = if corner_to_center.x > 0. && corner_to_center.y > 0. {
        // Corner case
        let corner_to_center_sqr = corner_to_center.length_squared();
        if corner_to_center_sqr > r * r {
            return None;
        }
        let corner_dist = corner_to_center_sqr.sqrt();
        let penetration = r - corner_dist;
        let n = corner_to_center / corner_dist * -s;
        (n, penetration)
    } else if corner_to_center.x > corner_to_center.y {
        // Closer to vertical edge
        (Vec2::X * -s.x, -corner_to_center.x + r)
    } else {
        (Vec2::Y * -s.y, -corner_to_center.y + r)
    };

    Some(Contact {
        normal: n,
        penetration,
    })
}

fn local_box_box(half_a: Vec2, ab: Vec2, rot_b: Rot, half_b: Vec2) -> Option<Contact> {
    let v1 = rot_b.rotate(Vec2::new(half_b.x, half_b.y));
    let v2 = rot_b.rotate(Vec2::new(half_b.x, -half_b.y));
    let v3 = -v1;
    let v4 = -v2;

    let v1 = v1 + ab;
    let v2 = v2 + ab;
    let v3 = v3 + ab;
    let v4 = v4 + ab;

    let mut min_penetration = f32::MAX;
    let mut n = Vec2::ZERO;
    let v_max = v1.max(v2).max(v3.max(v4));
    let v_min = v1.min(v2).min(v3.min(v4));

    // right edge
    {
        let penetration = half_a.x - v_min.x;
        if penetration < 0. {
            return None;
        } else if penetration < min_penetration {
            min_penetration = penetration;
            n = Vec2::X;
        }
    }

    // left edge
    {
        let penetration = half_a.x + v_max.x;
        if penetration < 0. {
            return None;
        } else if penetration < min_penetration {
            min_penetration = penetration;
            n = -Vec2::X;
        }
    }

    // top edge
    {
        let penetration = half_a.y - v_min.y;
        if penetration < 0. {
            return None;
        } else if penetration < min_penetration {
            min_penetration = penetration;
            n = Vec2::Y;
        }
    }

    // bottom edge
    {
        let penetration = half_a.y + v_max.y;
        if penetration < 0. {
            return None;
        } else if penetration < min_penetration {
            min_penetration = penetration;
            n = -Vec2::Y;
        }
    }

    Some(Contact {
        penetration: min_penetration,
        normal: n,
    })
}

pub fn box_box(
    pos_a: Vec2,
    rot_a: Rot,
    size_a: Vec2,
    pos_b: Vec2,
    rot_b: Rot,
    size_b: Vec2,
) -> Option<Contact> {
    let half_a = size_a / 2.;
    let half_b = size_b / 2.;
    let ab = pos_b - pos_a;
    let rot_ab = rot_a.inv().mul(rot_b);
    if let Some(a_contact) = local_box_box(half_a, rot_a.inv().rotate(ab), rot_ab, half_b) {
        // Check if there is a better separating axis along the other box' normals.
        if let Some(b_contact) =
        local_box_box(half_b, rot_b.inv().rotate(-ab), rot_ab.inv(), half_a)
        {
            if b_contact.penetration < a_contact.penetration {
                return Some(Contact {
                    penetration: b_contact.penetration,
                    normal: rot_b.rotate(-b_contact.normal),
                });
            }
        } else {
            return None;
        }
        Some(Contact {
            penetration: a_contact.penetration,
            normal: rot_a.rotate(a_contact.normal),
        })
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_box_clear() {
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::new(1.1, 0.), Default::default(), Vec2::ONE).is_none());
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::new(-1.1, 0.), Default::default(), Vec2::ONE).is_none());
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::new(0., 1.1), Default::default(), Vec2::ONE).is_none());
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::new(0., -1.1), Default::default(), Vec2::ONE).is_none());
    }

    #[test]
    fn box_box_intersection() {
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::ZERO, Default::default(), Vec2::ONE).is_some());
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::new(0.9, 0.9), Default::default(), Vec2::ONE).is_some());
        assert!(box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                        Vec2::new(-0.9, -0.9), Default::default(), Vec2::ONE).is_some());
    }

    #[test]
    fn box_box_contact() {
        let Contact {
            normal,
            penetration,
        } = box_box(Vec2::ZERO, Default::default(), Vec2::ONE,
                    Vec2::new(0.9, 0.), Default::default(), Vec2::ONE).unwrap();

        assert!(normal.x > 0.);
        assert!(normal.y < 0.001);
        assert!((penetration - 0.1).abs() < 0.001);
    }
}