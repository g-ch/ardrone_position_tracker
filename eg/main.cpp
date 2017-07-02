#include "mainwindow.h"
#include <QApplication>
#include "point_feature.h"
#include <iostream>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    vector<float> x;
    vector<float> y;

    x.push_back(0.0);
    y.push_back(0.0);
    x.push_back(1.0);
    y.push_back(1.0);
    x.push_back(-1.0);
    y.push_back(1.0);

    vector<vector<float> > v;
    p_feature_calculate(x,y,10.0,30,v);

    for(int i=0; i< 3;i++)
    {
        for(int j=0; j<30; j++)
            std::cout<< v[i][j] <<" ";

        std::cout<<endl;
    }

    vector<float> x1;
    vector<float> y1;

    x1.push_back(0.0);
    y1.push_back(0.0);
    x1.push_back(1.4);
    y1.push_back(-0.05);
    x1.push_back(0.0);
    y1.push_back(1.4);

    vector<vector<float> > v1;
    p_feature_calculate(x1,y1,10.0,30,v1);

    for(int i=0; i< 3;i++)
    {
        for(int j=0; j<30; j++)
            std::cout<< v1[i][j] <<" ";

        vector<float> cc = v1[i];
        vector_move_once(cc, false);
        vector_move_once(cc, false);

        std::cout<<"con:";

        for(int j=0; j<30; j++)
            std::cout<< cc[j] <<" ";

        std::cout<<endl;
    }


    float an1;
    float an2;
    cout<< p_feature_sdistance(v[0], v1[0], 0, an1) <<endl;
    cout<< an1 << endl;
    cout<< p_feature_sdistance(v[0], v1[0], 60, an2) <<endl;
    cout<< an2 << endl;


    return a.exec();
}
