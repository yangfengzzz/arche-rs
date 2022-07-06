use bevy::prelude::*;

use crate::*;

#[derive(Bundle, Default)]
pub struct ParticleBundle {
    pub pos: Pos,
    pub prev_pos: PrevPos,
    pub mass: Mass,
    pub collider: CircleCollider,
    pub vel: Vel,
    pub pre_solve_vel: PreSolveVel,
    pub restitution: Restitution
}

impl ParticleBundle {
    pub fn new_with_pos_and_vel(pos: Vec2, vel: Vec2) -> Self {
        Self {
            pos: Pos(pos),
            prev_pos: PrevPos(pos - vel * DELTA_TIME),
            vel: Vel(vel),
            ..Default::default()
        }
    }
}