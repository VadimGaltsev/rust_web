use quicksilver::{
    Result,
    geom::{Shape, Rectangle, Circle, Transform, Vector},
    graphics::{Background::Col, Color},
    lifecycle::{Settings, State, Window, run},
    input::Key,
};
use ncollide2d::query::point_internal::point_query::PointQuery;
use nalgebra::{Isometry2, Vector2, Unit};
use ncollide2d::{
    world::CollisionGroups,
    world::CollisionWorld,
    world::CollisionObject,
    world::GeometricQueryType,
    shape::ShapeHandle,
    world::CollisionObjectHandle,
    shape::Plane,
    math::*
};
use ncollide2d::narrow_phase::ContactAlgorithm;
use ncollide2d::events::ContactEvent;

use nalgebra::zero;
use std::cell::Cell;

struct DrawGeometry {
    view: Rectangle,
    ball: Circle,
    world: CollisionWorld<f32, CollisionObjectData>,
}

#[derive(Clone)]
struct CollisionObjectData {
    pub name: &'static str,
    pub velocity: Option<Cell<Vector2<f32>>>,
}

impl CollisionObjectData {
    pub fn new(name: &'static str, velocity: Option<Vector2<f32>>) -> CollisionObjectData {
        let init_velocity;
        if let Some(velocity) = velocity {
            init_velocity = Some(Cell::new(velocity))
        } else {
            init_velocity = None
        }

        CollisionObjectData {
            name: name,
            velocity: init_velocity,
        }
    }
}

use ncollide2d::*;
use nalgebra::*;
use ncollide2d::query::{ContactManifold, TrackedContact};


fn handle_contact_event(world: &CollisionWorld<f32, CollisionObjectData>, event: &ContactEvent) {
    if let &ContactEvent::Started(collider1, collider2) = event {
        // NOTE: real-life applications would avoid this systematic allocation.
        let pair = world.contact_pair(collider1, collider2, false).unwrap();
        let mut collector: Vec<&TrackedContact<f32>> = pair.3.contacts().collect();


        let co1 = world.collision_object(collider1).unwrap();
        let co2 = world.collision_object(collider2).unwrap();

        // The ball is the one with a non-None velocity.
        if let Some(ref vel) = co1.data().velocity {
            let normal = collector[0].contact.normal;
            vel.set(vel.get() - 2.0 * nalgebra::dot(&vel.get(), &normal) * *normal);
        }
        if let Some(ref vel) = co2.data().velocity {
            let normal = -collector[0].contact.normal;
            vel.set(vel.get() - 2.0 * nalgebra::dot(&vel.get(), &normal) * *normal);
        }
    }
}

impl State for DrawGeometry {
    fn new() -> Result<DrawGeometry> {
        let plane_left = ShapeHandle::new(Plane::new(Vector2::x_axis()));
        let plane_bottom = ShapeHandle::new(Plane::new(Vector2::y_axis()));
        let plane_right = ShapeHandle::new(Plane::new(-Vector2::x_axis()));
        let plane_top = ShapeHandle::new(Plane::new(-Vector2::y_axis()));
        let mut world = CollisionWorld::new(0.02);
        let contacts_query = GeometricQueryType::Contacts(0.0, 0.0);
        let proximity_query = GeometricQueryType::Proximity(0.0);
        let planes_pos = [
            Isometry2::new(Vector2::new(0.0, 600.), nalgebra::zero()),
            Isometry2::new(Vector2::new(800.0, 0.), nalgebra::zero()),
            Isometry2::new(Vector2::new(800.0, 0.0), nalgebra::zero()),
            Isometry2::new(Vector2::new(800.0, 600.0), nalgebra::zero())
        ];
        let mut others_groups = CollisionGroups::new();
        others_groups.set_membership(&[2]);
        others_groups.set_whitelist(&[1]);
        world.add(planes_pos[0],
                  plane_left, others_groups,
                  contacts_query,
                  CollisionObjectData::new("1", None));
        world.add(planes_pos[1],
                  plane_bottom, others_groups,
                  contacts_query,
                  CollisionObjectData::new("2", None));
        world.add(planes_pos[3],
                  plane_right, others_groups,
                  contacts_query,
                  CollisionObjectData::new("3", None));
        world.add(planes_pos[3],
                  plane_top, others_groups,
                  contacts_query,
                  CollisionObjectData::new("4", None));
        let mut ball = Circle::new((400, 400), 30);
        let mut ball_groups = CollisionGroups::new();
        ball_groups.set_membership(&[1]);
        let ball_handle = world.add(
            Isometry2::new(Vector2::new(400., 400.), nalgebra::zero()),
            ShapeHandle::new(ball.into_ball()),
            ball_groups, contacts_query,
            CollisionObjectData::new("ball", Some(Vector2::new(10.0, 5.0))));
        let scene = DrawGeometry {
            view: Rectangle::new(Vector::new(0.0, 0.0), Vector::new(100.0, 100.0)),
            ball,
            world,
        };
        Ok(scene)
    }

    fn update(&mut self, _window: &mut Window) -> Result<()> {
        if _window.keyboard()[Key::Up].is_down() {
            self.ball = self.ball.translate((0, -4));
        }
        if _window.keyboard()[Key::Down].is_down() {
            self.ball = self.ball.translate((0, 4));
        }
        if _window.keyboard()[Key::Left].is_down() {
            self.ball = self.ball.translate((-4, 0));
        }
        if _window.keyboard()[Key::Right].is_down() {
            self.ball = self.ball.translate((4, 0));
        }
//        let point = &self.view.pos.into_point();
//        println!("Current pos {:?}", point);
//        let dist = self.ball.bounding_box().into_cuboid().distance_to_point(
//            &Isometry2::identity(),
//            point,
//            true);
//        println!("is contact {}", dist);
        Ok(())
    }


    fn draw(&mut self, window: &mut Window) -> Result<()> {
        window.clear(Color::WHITE)?;
        for event in self.world.contact_events() {
            println!("{:?}", event);
        }
        let mut ball_handle = CollisionObjectHandle(1);
        for obg in self.world.collision_objects() {
            if obg.data().name == "ball" {
                ball_handle = obg.handle();
            }
        }

        let timestep = 0.016;

        for event in self.world.contact_events() {
            handle_contact_event(&self.world, event)
        }

        let ball_object = self.world.collision_object(ball_handle).unwrap();
        let ball_velocity = ball_object.data().velocity.as_ref().unwrap();
        let ball_pos = ball_object.position().append_translation(&(timestep * ball_velocity.get()));
        self.world.set_position(ball_handle,
                                Isometry2::new(
                                    Vector2::new(self.ball.pos.x, self.ball.pos.y),
                                    nalgebra::zero()));
        self.world.update();
        window.draw(&self.view, Col(Color::BLACK));
        window.draw(&self.ball, Col(Color::BLUE));
        Ok(())
    }
}

fn main() {
    run::<DrawGeometry>("Draw Geometry", Vector::new(800, 600), Settings::default());
}

//use nalgebra::{zero, Isometry2};
//use ncollide2d::query::{Ray, RayCast};
//use quicksilver::{
//    Result,
//    geom::{Rectangle, Vector},
//    graphics::{Color, GpuTriangle, Mesh, Vertex},
//    lifecycle::{Event, Settings, State, Window, run},
//};
//use std::cmp::Ordering;
//
//struct Raycast {
//    // the rectangles to raycast against
//    regions: Vec<Rectangle>,
//    // the points to send rays to
//    targets: Vec<Vector>,
//    // the vertices to draw to the screen
//    mesh: Mesh
//}
//
//impl State for Raycast {
//    fn new() -> Result<Raycast> {
//        //The different squares that cast shadows
//        let regions = vec![
//            Rectangle::new_sized((800, 600)),
//            // Feel free to add or remove rectangles to this list
//            // to see the effect on the lighting
//            Rectangle::new((200, 200), (100, 100)),
//            Rectangle::new((400, 200), (100, 100)),
//            Rectangle::new((400, 400), (100, 100)),
//            Rectangle::new((200, 400), (100, 100)),
//            Rectangle::new((50, 50), (50, 50)),
//            Rectangle::new((550, 300), (64, 64))
//        ];
//        // Build the list of targets to cast rays to
//        let targets = regions
//            .iter()
//            .flat_map(|region| {
//                vec![
//                    region.top_left(),
//                    region.top_left() + region.size().x_comp(),
//                    region.top_left() + region.size().y_comp(),
//                    region.top_left() + region.size(),
//                ].into_iter()
//            })
//            .collect();
//        Ok(Raycast {
//            regions,
//            targets,
//            mesh: Mesh::new(),
//        })
//    }
//
//    fn event(&mut self, event: &Event, window: &mut Window) -> Result<()> {
//        if let &Event::MouseMoved(_) = event {
//            let mouse = window.mouse().pos();
//            self.mesh.clear();
//            let distance_to = |point: &Vector| (*point - mouse).len();
//            let angle_to = |point: &Vertex| (point.pos - mouse).angle();
//            // Raycast towards all targets and find the vertices
//            for i in 0..self.targets.len() {
//                let angle = (self.targets[i] - mouse).angle();
//                let mut cast_ray = |direction: f32| {
//                    // Create a Ray from the mouse to the target
//                    let start = mouse.into_point();
//                    let direction = Vector::from_angle(direction).into_vector();
//                    let ray = Ray::new(start, direction);
//                    // Perform the actual raycast, returning the target and an iterator of collisions
//                    let identity = Isometry2::new(zero(), zero());
//                    let cast = self.regions
//                        .iter()
//                        .filter_map(|region| {
//                            region.into_aabb().toi_with_ray(&identity, &ray, false)
//                        })
//                        .map(|toi: f32| (ray.origin + toi * ray.dir).into())
//                        .min_by(|a: &Vector, b: &Vector| {
//                            distance_to(a)
//                                .partial_cmp(&distance_to(b))
//                                .unwrap_or(Ordering::Equal)
//                        });
//                    if let Some(pos) = cast {
//                        self.mesh.vertices.push(Vertex {
//                            pos,
//                            tex_pos: None,
//                            col: Color::WHITE,
//                        });
//                    }
//                };
//                // Make sure to cast rays around corners to avoid jitteriness
//                cast_ray(angle - 0.001);
//                cast_ray(angle);
//                cast_ray(angle + 0.001);
//            }
//            // Sort the vertices to form a visibility polygon
//            self.mesh.vertices.sort_by(|a, b| {
//                angle_to(a)
//                    .partial_cmp(&angle_to(b))
//                    .unwrap_or(Ordering::Equal)
//            });
//            // Insert the mouse as a vertex for the center of the polygon
//            self.mesh.vertices.insert(
//                0,
//                Vertex {
//                    pos: mouse,
//                    tex_pos: None,
//                    col: Color::WHITE,
//                },
//            );
//            // Calculate the number of triangles needed to draw the poly
//            let triangle_count = self.mesh.vertices.len() as u32 - 1;
//            for index in 0..triangle_count {
//                self.mesh.triangles.push(GpuTriangle {
//                    z: 0.0,
//                    indices: [
//                        0,
//                        index as u32 + 1,
//                        (index as u32 + 1) % triangle_count + 1
//                    ],
//                    image: None
//                });
//            }
//        }
//        Ok(())
//    }
//
//    fn draw(&mut self, window: &mut Window) -> Result<()> {
//        window.clear(Color::BLACK)?;
//        window.mesh().extend(&self.mesh);
//        Ok(())
//    }
//}
//
//fn main() {
//    run::<Raycast>("Raycast", Vector::new(800, 600), Settings::default());
//}