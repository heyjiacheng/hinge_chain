use glam::Vec3;
use crate::physics::RigidBody;

/// 铰链约束 - 将刚体上的一点固定在空间中的锚点
pub struct HingeConstraint {
    /// 世界空间的固定点
    pub anchor: Vec3,
    /// 刚体局部空间的连接点
    pub local_anchor: Vec3,
}

impl HingeConstraint {
    pub fn new(anchor: Vec3, local_anchor: Vec3) -> Self {
        Self {
            anchor,
            local_anchor,
        }
    }

    /// 求解位置约束 - 将刚体的连接点拉向固定锚点
    pub fn solve_position(&self, body: &mut RigidBody, strength: f32) {
        // 计算刚体连接点的世界位置
        let world_anchor = body.position + body.orientation * self.local_anchor;

        // 计算误差并修正
        let error = self.anchor - world_anchor;
        if error.length_squared() > 1e-8 {
            body.position += error * strength;
        }
    }

    /// 求解速度约束 - 轻微阻尼保持数值稳定
    pub fn solve_velocity(&self, body: &mut RigidBody, damping: f32) {
        // 计算连接点的速度
        let r = body.orientation * self.local_anchor;
        let anchor_velocity = body.linear_velocity + body.angular_velocity.cross(r);

        // 轻微修正，保留摆动动量
        if anchor_velocity.length_squared() > 1e-8 {
            body.linear_velocity -= anchor_velocity * damping;

            let angular_correction = r.cross(anchor_velocity) * damping;
            body.angular_velocity -= angular_correction / body.mass * 0.1;
        }
    }
}

/// 单摆系统
pub struct PendulumSystem {
    pub body: RigidBody,
    pub constraint: HingeConstraint,
    pub gravity: Vec3,
}

impl PendulumSystem {
    pub fn new(body: RigidBody, constraint: HingeConstraint, gravity: Vec3) -> Self {
        Self {
            body,
            constraint,
            gravity,
        }
    }

    /// 物理步进
    pub fn step(&mut self, dt: f32) {
        // 1. 施加重力
        self.body.apply_force(self.gravity * self.body.mass, dt);

        // 2. 积分运动
        self.body.integrate(dt);

        // 3. 求解约束（多次迭代提高稳定性）
        for _ in 0..3 {
            self.constraint.solve_position(&mut self.body, 0.5);
        }
        self.constraint.solve_velocity(&mut self.body, 0.1);
    }
}
