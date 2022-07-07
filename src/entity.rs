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
    pub restitution: Restitution,
    pub aabb: Aabb,
}

impl ParticleBundle {
    pub fn new_with_pos_and_vel(pos: Vec2, vel: Vec2) -> Self {
        Self {
            pos: Pos(pos),
            prev_pos: PrevPos(pos - vel * SUB_DT),
            vel: Vel(vel),
            ..Default::default()
        }
    }
}

//--------------------------------------------------------------------------------------------------
#[derive(Bundle, Default)]
pub struct DynamicBoxBundle {
    pub pos: Pos,
    pub rot: Rot,
    pub prev_pos: PrevPos,
    pub prev_rot: PrevRot,
    pub mass: Mass,
    pub collider: BoxCollider,
    pub vel: Vel,
    pub ang_vel: AngVel,
    pub pre_solve_vel: PreSolveVel,
    pub restitution: Restitution,
    pub aabb: Aabb,
}

impl DynamicBoxBundle {
    pub fn new_with_pos_and_vel_and_rot_and_ang_vel(
        pos: Vec2,
        vel: Vec2,
        rot: Rot,
        ang_vel: f32,
    ) -> Self {
        Self {
            rot,
            prev_rot: PrevRot(rot.mul(Rot::from_radians(-ang_vel * SUB_DT))),
            ang_vel: AngVel(ang_vel),
            ..Self::new_with_pos_and_vel(pos, vel)
        }
    }
}

//--------------------------------------------------------------------------------------------------
#[derive(Bundle, Default)]
pub struct StaticCircleBundle {
    pub pos: Pos,
    pub rot: Rot,
    pub collider: CircleCollider,
    pub restitution: Restitution,
}

#[derive(Bundle, Default)]
pub struct StaticBoxBundle {
    pub pos: Pos,
    pub rot: Rot,
    pub collider: BoxCollider,
    pub restitution: Restitution,
}