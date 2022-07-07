use bevy::prelude::*;

#[derive(Component, Debug, Default)]
pub struct Pos(pub Vec2);

#[derive(Component, Debug, Default)]
pub struct PrevPos(pub Vec2);

#[derive(Component, Debug, Default)]
pub struct PreSolveAngVel(pub(crate) f32);

#[derive(Component, Debug)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Self {
        Self(1.) // Default to 1 kg
    }
}

#[derive(Component, Debug)]
pub struct CircleCollider {
    pub radius: f32,
}

impl Default for CircleCollider {
    fn default() -> Self {
        Self { radius: 0.5 }
    }
}


#[derive(Component, Debug)]
pub struct BoxCollider {
    pub size: Vec2,
}

impl BoxCollider {
    pub fn inertia_inv_from_mass_inv(&self, mass_inv: f32) -> f32 {
        12. * mass_inv / self.size.length_squared()
    }
}

impl Default for BoxCollider {
    fn default() -> Self {
        Self { size: Vec2::ONE }
    }
}

#[derive(Component, Debug, Default)]
pub struct Vel(pub(crate) Vec2);

#[derive(Component, Debug, Default)]
pub struct PreSolveVel(pub(crate) Vec2);

#[derive(Component, Debug)]
pub struct Restitution(pub f32);

impl Default for Restitution {
    fn default() -> Self {
        Self(0.3)
    }
}

#[derive(Component, Debug, Default)]
pub struct Aabb {
    pub(crate) min: Vec2,
    pub(crate) max: Vec2,
}

impl Aabb {
    pub fn intersects(&self, other: &Self) -> bool {
        self.max.x >= other.min.x
            && self.max.y >= other.min.y
            && self.min.x <= other.max.x
            && self.min.y <= other.max.y
    }
}

#[derive(Component, Debug, Default, Clone)]
pub struct Inertia {
    pub inv: f32,
}