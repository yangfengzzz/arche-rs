use std::f32::consts::PI;
use std::ops::{Add, AddAssign};
use bevy::prelude::*;
use rand::distributions::Standard;
use rand::prelude::Distribution;

#[derive(Debug, Clone, Copy, PartialEq, Component)]
pub struct Rot {
    cos: f32,
    sin: f32,
}

impl Default for Rot {
    fn default() -> Self {
        Self::ZERO
    }
}

impl Rot {
    pub const ZERO: Self = Self { cos: 1., sin: 0. };

    pub fn from_radians(radians: f32) -> Self {
        Self {
            cos: radians.cos(),
            sin: radians.sin(),
        }
    }

    pub fn from_degrees(degrees: f32) -> Self {
        let radians = degrees.to_radians();
        Self::from_radians(radians)
    }

    pub fn as_radians(&self) -> f32 {
        f32::atan2(self.sin, self.cos)
    }

    pub fn rotate(&self, vec: Vec2) -> Vec2 {
        Vec2::new(
            vec.x * self.cos - vec.y * self.sin,
            vec.x * self.sin + vec.y * self.cos,
        )
    }

    pub fn inv(self) -> Self {
        Self {
            cos: self.cos,
            sin: -self.sin,
        }
    }

    pub fn mul(self, rhs: Rot) -> Self {
        Self {
            cos: self.cos * rhs.cos - self.sin * rhs.sin,
            sin: self.sin * rhs.cos + self.cos * rhs.sin,
        }
    }

    pub fn sin(self) -> f32 {
        self.sin
    }

    pub fn cos(self) -> f32 {
        self.cos
    }
}

impl Add<Self> for Rot {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        self.mul(rhs)
    }
}

impl AddAssign<Self> for Rot {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl From<Rot> for f32 {
    fn from(rot: Rot) -> Self {
        rot.as_radians()
    }
}

impl From<Rot> for Quat {
    fn from(rot: Rot) -> Self {
        if rot.cos < 0. {
            let t = 1. - rot.cos;
            let d = 1. / (t * 2.).sqrt();
            let z = t * d;
            let w = -rot.sin * d;
            Quat::from_xyzw(0., 0., z, w)
        } else {
            let t = 1. + rot.cos;
            let d = 1. / (t * 2.).sqrt();
            let z = -rot.sin * d;
            let w = t * d;
            Quat::from_xyzw(0., 0., z, w)
        }
    }
}

impl Distribution<Rot> for Standard {
    fn sample<R: rand::Rng + ?Sized>(&self, rng: &mut R) -> Rot {
        let radians = rng.gen_range::<f32, _>(-PI..PI);
        Rot::from_radians(radians)
    }
}

#[derive(Component, Debug, Default)]
pub struct PrevRot(pub Rot);

#[derive(Component, Debug, Default)]
pub struct AngVel(pub(crate) f32);