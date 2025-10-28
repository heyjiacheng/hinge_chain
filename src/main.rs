mod physics;
mod pendulum;

use bevy::prelude::*;
use glam::{Quat, Vec3};
use physics::RigidBody;
use pendulum::{HingeConstraint, PendulumSystem};

// ============ 常量配置 ============

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
const PHYSICS_TIMESTEP: f32 = 1.0 / 120.0;

const PENDULUM_LENGTH: f32 = 2.0;
const PENDULUM_MASS: f32 = 1.0;
const INITIAL_ANGLE: f32 = 55.0; // 度

const CAPSULE_RADIUS: f32 = 0.2;
const CAPSULE_HALF_HEIGHT: f32 = 0.5;

// ============ 组件和资源 ============

#[derive(Component)]
struct PendulumMarker;

#[derive(Resource)]
struct PhysicsWorld {
    pendulum: PendulumSystem,
    accumulator: f32,
}

// ============ 主函数 ============

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, (physics_update, sync_rendering))
        .run();
}

// ============ 初始化系统 ============

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 创建单摆物理系统
    let pendulum_system = create_pendulum_system();
    commands.insert_resource(PhysicsWorld {
        pendulum: pendulum_system,
        accumulator: 0.0,
    });

    // 渲染：胶囊体
    commands.spawn((
        Mesh3d(meshes.add(Capsule3d::new(CAPSULE_RADIUS, CAPSULE_HALF_HEIGHT * 2.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.3, 0.3))),
        Transform::default(),
        PendulumMarker,
    ));

    // 渲染：铰链点
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.1))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.8, 0.3))),
        Transform::from_translation(Vec3::ZERO),
    ));

    // 相机
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 0.0, 8.0).looking_at(Vec3::new(0.0, -1.0, 0.0), Vec3::Y),
    ));

    // 光照
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
    ));

    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        affects_lightmapped_meshes: false,
    });
}

// ============ 物理系统 ============

fn create_pendulum_system() -> PendulumSystem {
    // 创建刚体
    let mut body = RigidBody::new(PENDULUM_MASS);

    // 设置初始位置和角度
    let angle_rad = INITIAL_ANGLE.to_radians();
    body.position = Vec3::new(
        angle_rad.sin() * PENDULUM_LENGTH,
        -angle_rad.cos() * PENDULUM_LENGTH,
        0.0,
    );
    body.orientation = Quat::from_rotation_z(angle_rad);

    // 创建铰链约束
    let anchor = Vec3::ZERO;
    let local_anchor = Vec3::new(0.0, CAPSULE_HALF_HEIGHT + CAPSULE_RADIUS, 0.0);
    let constraint = HingeConstraint::new(anchor, local_anchor);

    PendulumSystem::new(body, constraint, GRAVITY)
}

fn physics_update(time: Res<Time>, mut physics_world: ResMut<PhysicsWorld>) {
    physics_world.accumulator += time.delta_secs();

    // 固定时间步长更新
    while physics_world.accumulator >= PHYSICS_TIMESTEP {
        physics_world.pendulum.step(PHYSICS_TIMESTEP);
        physics_world.accumulator -= PHYSICS_TIMESTEP;
    }
}

// ============ 渲染同步 ============

fn sync_rendering(
    physics_world: Res<PhysicsWorld>,
    mut query: Query<&mut Transform, With<PendulumMarker>>,
) {
    if let Ok(mut transform) = query.single_mut() {
        let body = &physics_world.pendulum.body;
        transform.translation = body.position;
        transform.rotation = body.orientation;
    }
}
