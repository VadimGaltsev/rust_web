use quicksilver::{
    Result,
    geom::{Rectangle, Circle, Vector, Shape},
    graphics::{Background::Col, Color},
    lifecycle::{Settings, State, Window, run},
    input::Key,
    lifecycle::Event,
};
use nalgebra as na;
use na::{Vector2, Isometry2, zero};
use ncollide2d::{
    shape::{ShapeHandle, Ball, Cuboid},
    world::CollisionObjectHandle,
    events::ContactEvent,
};
use nphysics2d::{
    world::World,
    object::{RigidBodyDesc, ColliderDesc, BodyStatus, Body},
    material::{MaterialHandle, BasicMaterial},
    algebra::Velocity2,
};
use ncollide2d::world::CollisionGroups;
use laminar::{
    Packet,
    Socket
};
use crossbeam::channel::Sender;
use byteorder::{ByteOrder, LittleEndian, BigEndian, WriteBytesExt, ReadBytesExt};

struct PlatformPlay {
    view: Rectangle,
    ball: Circle,
    world: World<f32>,

}

impl State for PlatformPlay {
    fn new() -> Result<PlatformPlay> {
        let rect_c = Rectangle::new((400., 20.), (40., 10.));
        let mut world = World::new();
        world.set_gravity(Vector2::new(0., 9.));
        let rect = ShapeHandle::new(Cuboid::new(Vector2::new(10., 300.)));
        let rect2 = ShapeHandle::new(Cuboid::new(Vector2::new(10., 300.)));
        let rect3 = ShapeHandle::new(Cuboid::new(Vector2::new(400., 10.)));
        let rect4 = ShapeHandle::new(Cuboid::new(Vector2::new(400., 10.)));
        let parent_handle = RigidBodyDesc::new();
        let parent_handle2 = RigidBodyDesc::new();
        let wall_group = MaterialHandle::new(BasicMaterial::new(0.7, 0.4));
        let handle_box = ColliderDesc::new(rect)
            .position(Isometry2::new(Vector2::new(0.0, -300.0), 0.))
            .material(wall_group.clone());
        let handle_box2 = ColliderDesc::new(rect2)
            .position(Isometry2::new(Vector2::new(800.0, -300.0), 0.))
            .material(wall_group.clone());
        let handle_box23 = ColliderDesc::new(rect3)
            .position(Isometry2::new(Vector2::new(400.0, 0.0), 0.))
            .material(wall_group.clone());
        let handle_box4 = ColliderDesc::new(rect4)
            .position(Isometry2::new(Vector2::new(400.0, -600.0), 0.))
            .material(wall_group.clone());
        let shape = ShapeHandle::new(Ball::new(10.));
        let collider = ColliderDesc::new(shape)
            .position(Isometry2::new(Vector2::new(400.0, -300.0), 0.))
            .name("ball".to_owned())
            .material(wall_group.clone())
            .density(0.001)
            .user_data(10);
        parent_handle2
            .collider(&collider)
            .build(&mut world);
        parent_handle.collider(&handle_box)
            .name("Walls".to_owned())
            .gravity_enabled(false)
            .collider(&handle_box2)
            .collider(&handle_box23)
            .collider(&handle_box4)
            .build(&mut world);

        let platform = RigidBodyDesc::new()
            .gravity_enabled(false)
            .name("Pl body".to_string());
        let pl_shape = ShapeHandle::new(Cuboid::new(Vector2::new(20., 5.)));
        let pl_collider = ColliderDesc::new(pl_shape)
            .material(MaterialHandle::new(BasicMaterial::new(1.5, 0.4)))
            .name("pl1".to_owned())
            .density(10.)
            .position(Isometry2::new(Vector2::new(415., -15.), 0.));
        let mut body_p = platform.collider(&pl_collider).build(&mut world);
        body_p.set_translations_kinematic(Vector2::new(false, true));
        let ball = Circle::new((400., 300.), 10.);

        /////
        let scene = PlatformPlay {
            view: rect_c,
            ball,
            world,
        };
        Ok(scene)
    }


    fn update(&mut self, _window: &mut Window) -> Result<()> {
        let mut chandle = CollisionObjectHandle(1);
        for collider in self.world.colliders() {
            if collider.name() == "ball" {
                self.ball.pos.x = collider.position().translation.vector.data[0];
                self.ball.pos.y = -collider.position().translation.vector.data[1];
            }
            if collider.name() == "pl1" {
                chandle = collider.handle();
            }
        }

        let mut body_handle = None;
        for b in self.world.bodies_with_name("Pl body") {
            body_handle = Some(b.handle());
        }

        let body = &mut if let Some(handle) = body_handle {
            Some(self.world.rigid_body_mut(handle).unwrap())
        } else {
            None
        };

        if _window.keyboard()[Key::Left].is_down() {
            if let Some(b) = body {
                b.set_velocity(Velocity2::linear(-120., zero()));
            }
        }
        if _window.keyboard()[Key::Right].is_down() {
            if let Some(b) = body {
                b.set_velocity(Velocity2::linear(120., zero()));
            }
        }
        let body = self.world.collider(chandle).unwrap();
        let body_pos: &Isometry2<f32> = body.position();
        self.view.pos.x = body_pos.translation.vector.data[0] - 20.;

//        if _window.keyboard()[Key::Down].is_down() {
//            self.world.set_gravity(Vector2::new(-1., -1.));
//        }
//        if _window.keyboard()[Key::Up].is_down() {
//            self.world.set_gravity(Vector2::new(1., 1.));
//        }
        Ok(())
    }

    fn draw(&mut self, window: &mut Window) -> Result<()> {
        window.clear(Color::WHITE)?;
        self.world.step();
        window.draw(&self.view, Col(Color::BLACK));
        window.draw(&self.ball, Col(Color::BLUE));
        window.draw(&Rectangle::new((0., 0.), (10., 600.)), Col(Color::BLACK));
        window.draw(&Rectangle::new((790., 0.0), (10., 800.)), Col(Color::BLACK));
        window.draw(&Rectangle::new((0.0, 0.0), (800., 10.)), Col(Color::BLACK));
        window.draw(&Rectangle::new((0.0, 590.0), (800., 10.)), Col(Color::BLACK));
        Ok(())
    }
}

fn main() {
//    use nphysics_testbed2d::Testbed;
//    let geom: PlatformPlay = PlatformPlay::new().unwrap();
//    let mut test = Testbed::new(geom.world);
//    test.run();

    run::<PlatformPlay>("Draw Geometry", Vector::new(800, 600),
                        Settings::default());
}