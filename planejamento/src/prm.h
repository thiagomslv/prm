#ifndef __PRM_CLASS__
#define __PRM_CLASS__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <queue>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "plan_interfaces/msg/target.hpp"
#include "std_msgs/msg/string.hpp"

#define NUM_NODES 200

using namespace cv;
using namespace boost;

using std::placeholders::_1;

class PRM : public rclcpp::Node{

    private:

        typedef struct NPoint{

            int x, y;
            float distance;
        }NPoint;

        typedef struct DPoint{

            int x, y;
            int label;

        }DPoint;

        // Definindo o tipo do grafo com pesos não negativos
        typedef adjacency_list<vecS, vecS, directedS, no_property, property<edge_weight_t, int>> Graph;
        typedef graph_traits<Graph>::vertex_descriptor Vertex;
        typedef graph_traits<Graph>::edge_descriptor Edge;

        rclcpp::Subscription<plan_interfaces::msg::Target>::SharedPtr subscription_;
        rclcpp::Publisher<plan_interfaces::msg::Target>::SharedPtr publisher_;

        int x_initial, y_initial;
        int x_final, y_final;

        Mat map;
        std::set<float> points;

        void readStoreAndProcessMap();

        void generateCoords(int numNodes);
        void checkIfPointIsInsideBarrier();

        void drawPointsAndGenerateMap();
        void knn(int k);

        void connectNeighboursAndVerifyConnections();
        bool rasterize_line(int x0, int y0, int x1, int y1);

        void dijkstra();

        void topic_callback(const plan_interfaces::msg::Target & msg);

    public:

        PRM()
        : Node("planejamento"){
            
            subscription_ = this->create_subscription<plan_interfaces::msg::Target>(
                "req_route", 10, std::bind(&PRM::topic_callback, this, _1));

            publisher_ = this->create_publisher<plan_interfaces::msg::Target>("route", 10);

        }

        void prm(int numNodes, int k);
};


#endif