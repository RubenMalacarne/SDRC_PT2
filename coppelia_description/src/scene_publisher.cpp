#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// Per forme/mesh
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

class ScenePublisher
{
public:
  // Costruttore: riceve un nodo e il nome del parametro URDF
  ScenePublisher(const rclcpp::Node::SharedPtr& node,
                 const std::string& robot_description = "robot_description",
                 const std::string& psm_name = "planning_scene_monitor")
    : node_(node)
  {
    // 1) Creiamo il PlanningSceneMonitor passando:
    //    - il nodo
    //    - il nome del parametro con l’URDF (es. "robot_description")
    //    - un nome arbitrario per il monitor
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, robot_description, psm_name
    );

    // 2) Verifichiamo che la scena sia stata caricata correttamente
    if (!psm_->getPlanningScene())
    {
      RCLCPP_ERROR(node_->get_logger(), 
                   "PlanningSceneMonitor non pronto. Assicurati che '%s' sia caricato!",
                   robot_description.c_str());
      return;
    }

    // 3) Creiamo un publisher su "/planning_scene"
    planning_scene_pub_ =
      node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

    // 4) Impostiamo un timer che fra 2 secondi chiamerà publishScene()
    timer_ = node_->create_wall_timer(std::chrono::seconds(2),
                                      std::bind(&ScenePublisher::publishScene, this));
  }

private:
  void publishScene()
  {
    // a) Recupera la PlanningScene corrente
    auto current_scene = psm_->getPlanningScene();
    if (!current_scene)
    {
      RCLCPP_ERROR(node_->get_logger(), "Impossibile accedere alla PlanningScene!");
      return;
    }

    // b) Copia la AllowedCollisionMatrix (contiene la self-collision del robot)
    collision_detection::AllowedCollisionMatrix acm =
      current_scene->getAllowedCollisionMatrix();

    // c) Aggiungi la regola: collisione permessa tra "base_link_inertia" e "table"
    acm.setEntry("base_link_inertia", "table", true);
    acm.setEntry("box", "table", true);

    // d) Converti l’ACM in un messaggio e lo metti in un PlanningScene diff
    moveit_msgs::msg::PlanningScene scene_msg;
    scene_msg.is_diff = true;
    acm.getMessage(scene_msg.allowed_collision_matrix);

    // e) Creiamo (o aggiorniamo) un CollisionObject per il tavolo
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world"; // Cambia se il tuo frame base è diverso
    table.operation = table.ADD;

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;

    // Carichiamo la mesh
    std::string package_path = ament_index_cpp::get_package_share_directory("coppelia_description");
    std::string mesh_path = package_path + "/meshes/lab_table_mesh.stl";

    shapes::Mesh* shape_mesh = shapes::createMeshFromResource("file://" + mesh_path);
    if (!shape_mesh)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Impossibile caricare la mesh da: %s", mesh_path.c_str());
      return;
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(shape_mesh, shape_msg);
    shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    table.meshes.push_back(mesh_msg);
    table.mesh_poses.push_back(table_pose);

    // Aggiungiamo il tavolo nella scena diff
    scene_msg.world.collision_objects.push_back(table);

    // Esempio: aggiungiamo anche un box
    moveit_msgs::msg::CollisionObject box;
    box.id = "box";
    box.header.frame_id = "world";
    box.operation = box.ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.05, 0.05, 0.15};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.900;
    box_pose.position.y = 0.625;
    box_pose.position.z = 0.939;

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(box_pose);

    scene_msg.world.collision_objects.push_back(box);

    // f) Pubblica la scena come "diff"
    planning_scene_pub_->publish(scene_msg);

    // g) Ferma il timer, così pubblichi solo una volta
    timer_->cancel();

    RCLCPP_INFO(node_->get_logger(),
                "Pubblicata scena diff con allowed collision base_link_inertia - table");
  }

  // Membro per conservare il nodo
  rclcpp::Node::SharedPtr node_;
  // Il PlanningSceneMonitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  // Publisher su /planning_scene
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  // Timer per chiamare publishScene() dopo un paio di secondi
  rclcpp::TimerBase::SharedPtr timer_;
};

// ---- MAIN ----
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 1) Crea un nodo
  auto node = rclcpp::Node::make_shared("scene_publisher_node");

  // 2) Istanzia ScenePublisher, passandogli il nodo e il nome del parametro URDF
  ScenePublisher sp(node, "robot_description", "my_psm");

  // 3) Esegui lo spin (finché non chiudi)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
