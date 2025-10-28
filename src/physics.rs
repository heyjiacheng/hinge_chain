use glam::{Quat, Vec3};

/// 刚体物理状态
#[derive(Debug, Clone)]
pub struct RigidBody {
    /// 位置（质心）
    pub position: Vec3,
    /// 旋转（四元数）
    pub orientation: Quat,
    /// 线速度
    pub linear_velocity: Vec3,
    /// 角速度
    pub angular_velocity: Vec3,
    /// 质量
    pub mass: f32,
}

impl RigidBody {
    /// 创建新刚体
    pub fn new(mass: f32) -> Self {
        Self {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass,
        }
    }

    /// 应用力（F = ma）
    pub fn apply_force(&mut self, force: Vec3, dt: f32) {
        self.linear_velocity += force / self.mass * dt;
    }

    /// 半隐式欧拉积分
    pub fn integrate(&mut self, dt: f32) {
        // 更新位置
        self.position += self.linear_velocity * dt;

        // 更新旋转
        if self.angular_velocity.length_squared() > 1e-6 {
            let angle = self.angular_velocity.length() * dt;
            let axis = self.angular_velocity.normalize();
            let delta_q = Quat::from_axis_angle(axis, angle);
            self.orientation = (delta_q * self.orientation).normalize();
        }
    }
}
