#include "prm.h"

void PRM::topic_callback(const plan_interfaces::msg::Target & msg){

    points.clear();

    x_initial = msg.cx, y_initial = msg.cy;
    x_final = msg.gx, y_final = msg.gy;

    float x = (float) x_initial;
    float y = (float) y_initial;

    //c = transforma em mantissa o y. d = soma a mantissa com o valor de x.
    float c = y / 1000, d = x + c;

    points.insert(d);

    x = (float) x_final;
    y = (float) y_final;

    c = y / 1000, d = x + c;

    points.insert(d);

    //+2 por causa dos pontos inicial e final que foram adicionados.
    prm(NUM_NODES + 2, 3);

    //RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg.cx);
}

void PRM::dijkstra(){

    //Precisamos definir uma matriz para armazenar os labels dos pontos.
    std::vector<DPoint> vert;
    std::vector<std::vector<int>> adjacents;
    int k = 0;

    for(auto& itr : points){

        DPoint p;

        int x = trunc(itr);
        float c = (itr - (float) x) * 1000;
        int y = round(c);

        p.x = x;
        p.y = y;
        p.label = k;

        vert.push_back(p);
        k++;
    }
    
    std::ifstream g("/home/thiago/ros2_ws/src/planejamento/src/files/graph.txt");
    std::string data;

    //std::vector<Spot> u_graph;
    std::vector<int> u_adj;

    int nVizinhos = 0;

    while(g){

        std::getline(g, data);

        if(data == "") continue;

        if(data == "-"){
            
            adjacents.push_back(u_adj);
            u_adj.clear();

        }else{

            std::size_t pos = data.find(" ");

            std::string x = data.substr (0, pos);
            std::string y = data.substr (pos + 1);

            //Encontra o label correspondente do ponto.
            int ux = stoi(x);
            int uy = stoi(y);

            int label = -1;

            for(int i = 0; i < vert.size(); i++){

                if(vert[i].x == ux && vert[i].y == uy){

                    label = vert[i].label;
                    break;
                }
            }

            u_adj.push_back(label);
            nVizinhos++;
        }
    }

    /*for(int i = 0; i < adjacents.size(); i++){

        for(int j = 0; j < adjacents[i].size(); j++){

            std::cout << adjacents[i][j] << std::endl;
        }

        std::cout << "-" << std::endl;
    }*/

    g.close();

    // Criação do grafo com pesos não negativos
    Graph G(nVizinhos);

    std::vector<DPoint> connections;
    
    for(int i = 0; i < adjacents.size(); i++){

        for(int j = 1; j < adjacents[i].size(); j++){

            int v1 = adjacents[i][0], v2 = adjacents[i][j];

            DPoint p1 = vert[v1], p2 = vert[v2];
            float distance = pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
            int weight = trunc(sqrt(distance));

            bool permited = true;

            for(int i = 0; i < connections.size(); i++){

                if((connections[i].x == v1 && connections[i].y == v2) || 
                    (connections[i].y == v1 && connections[i].x == v2)){

                        permited = false;
                    }
            }

            if(permited){

                DPoint d;
                d.x = v1;
                d.y = v2;

                //std::cout << "Adjacentes = " << v1 << " " << v2 << std::endl;
                connections.push_back(d);
                add_edge(v1, v2, weight, G);
                add_edge(v2, v1, weight, G);
            }
        }
    }

    // Vetor para armazenar as distâncias mínimas a partir do vértice 0
    std::vector<int> dist(num_vertices(G));

    int label = -1;

    for(int i = 0; i < vert.size(); i++){

        if(vert[i].x == x_initial && vert[i].y == y_initial){

            label = vert[i].label;
            break;
        }
    }

    Vertex start = label;

    // Executa o algoritmo de Dijkstra
    dijkstra_shortest_paths(G, start, distance_map(&dist[0]));

    // Encontra o caminho mais curto do vértice 0 ao vértice 4
    std::vector<Vertex> predecessors(num_vertices(G));

    label = -1;

    for(int i = 0; i < vert.size(); i++){

        if(vert[i].x == x_final && vert[i].y == y_final){

            label = vert[i].label;
            break;
        }
    }

    Vertex end = label;
    dijkstra_shortest_paths(G, start, predecessor_map(&predecessors[0]));

    // Imprime o caminho mais curto
    std::cout << "Caminho mais curto do vértice " << start << " ao vértice " << end << ":" << std::endl;
    std::vector<Vertex> path;

    int numCtrV = 0;
    bool permitedSubs = true;
    Vertex w = end;

    for (Vertex v = end; ; v = predecessors[v], numCtrV++)
    {
        if(numCtrV == 1){

            if(v == w){

                permitedSubs = false;
                break;
            } 
        }

        path.push_back(v);
        if (v == start)
            break;
    }

    if(permitedSubs){

        auto message = plan_interfaces::msg::Target();

        int sizep = path.size() - 1;

        for (int i = sizep; i >= 0; --i)
        {


            int l = path[i];
            std::cout << vert[l].x << " " << vert[l].y << " - ";


            if(i == sizep){

                message.cx = vert[l].x;
                message.cy = vert[l].y;
            }else if(i == sizep - 1){

                message.gx = vert[l].x;
                message.gy = vert[l].y;
            }
            
        }
        std::cout << std::endl;

        

        

        

        publisher_->publish(message);
    }
    
}

bool PRM::rasterize_line(int x0, int y0, int x1, int y1)
{   

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    Vec3b bgrPixel;

    while (x0 != x1 || y0 != y1)
    {
        //std::cout << "(" << x0 << "," << y0 << ") ";

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }

        bgrPixel = map.at<Vec3b>(x0, y0);

        //Pixel cor preta significa colisão.
        if(bgrPixel[0] == 0 && bgrPixel[1] == 0 && bgrPixel[2] == 0){

            return true;
        }
    }

    //std::cout << "(" << x1 << "," << y1 << ")" << std::endl;
    return false;
}

void PRM::connectNeighboursAndVerifyConnections(){

    std::ifstream neighbours("/home/thiago/ros2_ws/src/planejamento/src/files/neighbours.txt");
    std::ofstream graph("/home/thiago/ros2_ws/src/planejamento/src/files/graph.txt");

    int k = 0, i = 0;

    std::string data;
    std::getline(neighbours, data);

    k = stoi(data);
    
    while(neighbours){

        std::getline(neighbours, data);

        NPoint p, w;

        std::size_t pos = data.find(" ");

        std::string x = data.substr (0, pos);
        std::string y = data.substr (pos + 1);

        //Apenas para pegar o primeiro caso.
        if(i == 0){

            p.x = stoi(x);
            p.y = stoi(y);

            graph << p.x << " " << p.y << std::endl; 
            i++;
            
        } else if(i <= k){

            w.x = stoi(x);
            w.y = stoi(y);

            if(!rasterize_line(p.x, p.y, w.x, w.y)){

                graph << w.x << " " << w.y << std::endl;
                line(map, Point(p.y, p.x), Point(w.y, w.x), (0, 0, 255));
            }

            i++;
        }else{

            p.x = stoi(x);
            p.y = stoi(y);

            graph << "-" << std::endl;
            graph << p.x << " " << p.y << std::endl;

            i = 1;
        }
    }

    graph << "-" << std::endl;
    neighbours.close();
    graph.close();
}

void PRM::knn(int k){

    int n = points.size();

    std::vector<NPoint> vectors;
    std::vector<NPoint> arr;

    //Vetor para armazenar os vizinhos.
    std::vector<std::vector<NPoint>> neighbours;

    for(auto& item : points){

        int x = trunc(item);

        float c = (item - (float) x) * 1000;
        int y = round(c);

        NPoint p = {.x = x, .y=y};
        vectors.push_back(p);
    }

    //Calcula as distâncias do ponto selecionado com todos os outros.
    for(int i = 0; i < n; i++){

        NPoint p = vectors[i];

        for(int j = 0; j < n; j++){

            NPoint w = vectors[j];
            w.distance = sqrt( pow((w.x) - p.x, 2) +  pow((w.y) - p.y, 2));
            
            arr.push_back(w);
        }

        //Faz o sort para pegar os menores vizinhos.
        sort(arr.begin(), arr.end(), [](NPoint a, NPoint b){

            return (a.distance < b.distance);
        });

        neighbours.push_back(arr);
        arr.clear();
    }

    //Salva os vizinhos no arquivo de rotas.
    std::ofstream arq("/home/thiago/ros2_ws/src/planejamento/src/files/neighbours.txt");
    arq << k;

    for(int i = 0; i < n; i++){

        for(int j = 0; j <= k; j++){

            arq << std::endl << neighbours[i][j].x << " " << neighbours[i][j].y;
        }
    }

    arq.close();
}

void PRM::drawPointsAndGenerateMap(){

    Mat world = map.clone();

    for(auto& itr : points){

        int x = trunc(itr);
        float c = (itr - (float) x) * 1000;
        int y = round(c);

        circle(world, Point(y, x), 1, Scalar(0, 0, 255), 2);
    }

    imwrite("/home/thiago/ros2_ws/src/planejamento/src/files/worldPoints.png", world);
}

void PRM::checkIfPointIsInsideBarrier(){

    std::set<float> points_verified;
    std::set<float>::iterator itr;

    int width = map.size().width, height = map.size().height;
    int size = points.size();

    //Modifica a semente de aleatoriedade.
    srand(time(0));

    for(itr = points.begin(); itr != points.end(); itr++){

        float p =  *itr;
        int x = trunc(p);

        float c = (p - (float) x) * 1000;
        int y = round(c);

        int dif = size - points_verified.size();
        bool notSolutionFound = true;

        while(notSolutionFound){

            Vec3b bgrPixel = map.at<Vec3b>(x, y);

            //Pixel cor preta significa colisão.
            if(bgrPixel[0] == 0 && bgrPixel[1] == 0 && bgrPixel[2] == 0){

                x = rand() % width;
                y = rand() % height;

            }else{

                //Processa o ponto.
                //c = transforma em mantissa o y. d = soma a mantissa com o valor de x.
                float c = (float) y / 1000, d = (float) x + c;

                //Pixel livre, solução encontrada.
                points_verified.insert(d);
                int n_dif = size - points_verified.size();

                if(dif != n_dif){

                    notSolutionFound = false;
                }else{
                    
                    //O ponto selecionado pode não ter colisão, mas pode já existir
                    //dentro do set de pontos. Por isso, precisamos randomizar outro
                    //ponto.
                    x = rand() % width;
                    y = rand() % height;
                }
            }
        }
    }

    //Copia para o set principal de pontos.
    points = points_verified;

}

void PRM::generateCoords(int numNodes){

    //Altera a semente de aleatoriedade.
    srand(time(0));

    std::cout << "Gen coords = " << points.size() << std::endl;

    int width = map.size().width, height = map.size().height;

    /*Os pontos estarão contidos dentro de um único float: a parte após a vírgula
    é o ponto y e a parte decimal é o x.*/
    while(points.size() < numNodes){

        float x = (float) (rand() % width);
        float y = (float) (rand() % height);

        //c = transforma em mantissa o y. d = soma a mantissa com o valor de x.
        float c = y / 1000, d = x + c;

        points.insert(d);
    }
}

void PRM::readStoreAndProcessMap(){

    map = imread("/home/thiago/ros2_ws/src/planejamento/src/files/map.png");

    std::cout << map.empty() << std::endl;

    Mat map_binary(map.size(), map.type());
    threshold(map, map_binary, 0, 255, cv::THRESH_BINARY_INV);

    Mat map_dilated;

    //O kernel controla o tamanho da ditalação feita.
    //Ajustar depois para as dimensões do robô.
    Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10), Point(-1, 1));
    dilate(map_binary, map_dilated, kernel, Point(-1, 1), 1);

    //Reverte a imagem para os obstáculos permanecerem em preto.
    threshold(map_dilated, map_dilated, 100, 255, cv::THRESH_BINARY_INV);

    map = map_dilated;

    imwrite("/home/thiago/ros2_ws/src/planejamento/src/files/grid.png", map_dilated);
}

//Método mestre que vai chamar as demais funções.
void PRM::prm(int numNodes, int k){

    //Preparação da imagem para o processamento.
    readStoreAndProcessMap();

    //Gera n coordenadas de acordo com a quantidade de nós que foi repassado
    //e armazena no vetor pontos.
    generateCoords(numNodes);

    //Verifica se os pontos caíram em obstáculos.
    checkIfPointIsInsideBarrier();

    //Roda o KNN.
    knn(k);

    //Faz as conexões para formar o grafo.
    connectNeighboursAndVerifyConnections();

    //Desenha os pontos.
    drawPointsAndGenerateMap();

    //Determina o melhor caminho utilizando o dijkstra.
    dijkstra();
}