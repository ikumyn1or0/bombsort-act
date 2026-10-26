// 標準入力の1行「name x y z」ごとに /world/default/set_pose を非同期で呼ぶ中継。
// ros_gz_bridge（humble 版）が SetEntityPose のサービスを中継できないための代わり。
// 応答ごとに「ack 連番 成否 遅れms」を標準出力に出す。
// あわせて /world/default/dynamic_pose/info から名前が bomb_ で始まるモデルの世界座標を「pose 名前 x y z」で出す
// （PosePublisher は Fortress では最上位モデルの世界座標を出さず、Pose_V→TFMessage の中継は名前が落ちるため）。
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>

int main()
{
  ignition::transport::Node node;
  std::mutex mtx;
  std::map<int, std::chrono::steady_clock::time_point> sent;
  int seq = 0;
  std::function<void(const ignition::msgs::Pose_V &)> poseCb = [&](const ignition::msgs::Pose_V &v) {
    std::lock_guard<std::mutex> l(mtx);
    for (const auto &p : v.pose())
      if (p.name().rfind("bomb_", 0) == 0)
        std::cout << "pose " << p.name() << " " << p.position().x() << " " << p.position().y() << " " << p.position().z() << std::endl;
  };
  node.Subscribe("/world/default/dynamic_pose/info", poseCb);
  std::string line;
  std::ios::sync_with_stdio(false);
  while (std::getline(std::cin, line))
  {
    std::istringstream in(line);
    std::string name; double x, y, z;
    if (!(in >> name >> x >> y >> z)) continue;
    ignition::msgs::Pose req;
    req.set_name(name);
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_w(1.0);
    int id = ++seq;
    { std::lock_guard<std::mutex> l(mtx); sent[id] = std::chrono::steady_clock::now(); }
    std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [&, id](const ignition::msgs::Boolean &rep, const bool ok) {
        std::lock_guard<std::mutex> l(mtx);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sent[id]).count();
        std::cout << "ack " << id << " " << (ok && rep.data()) << " " << ms << std::endl;
      };
    node.Request("/world/default/set_pose", req, cb);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return 0;
}
