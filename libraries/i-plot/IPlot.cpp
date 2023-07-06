#include "IPlot.h"

IPlot::IPlot(double sampleTime, string new_title, string new_xLabel, string new_yLabel)
{
    Ts=sampleTime;
    x.clear();
//    x.push_back(0.);
    y.clear();
//    y.push_back(0.);
    N=0;
    xLabel = "'"+new_xLabel+"'"; //labels in gnuplot need the ' '.
    yLabel = "'"+new_yLabel+"'"; //labels in gnuplot need the ' '.
    title = "'"+new_title+"'"; //labels in gnuplot need the ' '.
    figure = GNUPLOT_POPEN("gnuplot -persistent", "w");
//    figdata = GNUPLOT_POPEN("gnuplot", "w");

}

IPlot::IPlot(vector<double> & new_y) : IPlot()
{
    y=new_y;
    N=y.size();
    x.resize(N);
    for (int i = 0; i < N; ++i)
    {
        x[i]=i;
    }


}

IPlot::~IPlot()
{
    GNUPLOT_PCLOSE(figure);

}


long IPlot::SetParameters(string new_parameters)
{
    parameters = new_parameters;

    return 0;
}

long IPlot::pushBack(double new_value)
{
    y.push_back(new_value);
    N++;
    x.push_back(N*Ts);

    return 0;
}

long IPlot::Update(double new_value)
{
    y.push_back(new_value);
    N++;
    x.push_back(N*Ts);

    fprintf(figure,"%f %f \n",N*Ts,new_value);
    fprintf(figure,"reread \n");
    return 0;
}

long IPlot::Plot()
{


//    xMax = *max_element(x.begin(),x.end());
//    yMax = *max_element(y.begin(),y.end());
    Plot(x,y,0,0);
    return 0;
}

long IPlot::Save(std::string filename)
{

    std::fstream datafile;
    datafile.open (filename, std::fstream::out);

    for (ulong i=1; i<x.size(); i++)
    {

        datafile << x[i] << " - "
                 << y[i] << ""

                 << std::endl;


    }
    cout << "Saved at: " << filename << endl;

    datafile.close();
    return 0;
}

/* old version using plotter
long IPlot::Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley)
{
    PlotterParams newParams;
    newParams.setplparam("PAGESIZE", (char *)"a4");
    newParams.setplparam("BITMAPSIZE", (char *)"600x600");

    XPlotter plt(newParams);
    plt.fspace(-scalex, scalex, -scaley, scaley);
    //plt.fscale(2,2);
    plt.openpl();
    plt.pencolorname("blue");




    for (ulong i=1; i<datax.size(); i++)
    {
        //plt.fpoint(datax[i]/scalex,datay[i]/scaley);
        plt.fmove(datax[i]/scalex,datay[i]/scaley);
        plt.fline(datax[i-1]/scalex,datay[i-1]/scaley,datax[i]/scalex,datay[i]/scaley);
        //plt.fcircle(datax[i]/scalex,datay[i]/scaley,std::max(scalex,scaley)/10000.);

        //plt.endpath();
        //plt.flushpl();

    }

    //    plt.move(0.5*scalex,0.5*scaley);
        sprintf (yLabel, "  yMax: %f ", scaley);
        sprintf (xLabel, " 'x y Label' ");

    //    cout << scalex;
        plt.label(yLabel);

    plt.endpath();
    plt.flushpl();
    plt.closepl();


    return 0;

}*/

long IPlot::Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley)
{


//        sprintf (yLabel, "  yMax: %f ", scaley);
//        sprintf (xLabel, " 'x y Label' ");


    fprintf(figure,"%s \n","set grid");

    fprintf(figure,"%s%s\n %s%s\n","set ylabel ", yLabel.c_str(), "set xlabel ", xLabel.c_str() );
    fprintf(figure,"%s%s\n ","set title ", title.c_str() );

    fprintf(figure,"%s \n",parameters.c_str());
//    fprintf(figure,"%s \n","set style line 1 linewidth 30");

    fprintf(figure,"%s\n","plot '-' lt rgb 'blue' with lines");
    for (int i = 0; i < datay.size(); ++i)
    {
        fprintf(figure,"%f %f \n",datax[i],datay[i]);
    }
    fprintf(figure,"%s\n","EOF");


    return 0;

}

long IPlot::PlotAndSave(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley, std::string filename)
{

    std::fstream datafile;
    datafile.open (filename, std::fstream::out);

    for (ulong i=1; i<datax.size(); i++)
    {


        //fprintf (gdata, "%f - %f - %f - %f \n",Ts*i,vel,actualVel,jointPos);
        datafile << datax[i] << " - "
                 << datay[i] << ""

                 << std::endl;


    }


    Plot(datax,datay,scalex,scaley);

    datafile.close();


    return 0;
}


long IPlot::PlotAndSave(string filename)
{

    Plot();
    Save(filename);

    return 0;
}
/* unused??
long IPlot::PlotAxis(double scalex, double scaley, double offset)
{

char yMarker[100];

plt.fmove(offset,offset);
    for (ulong i=1; i<10; i++)
    {
        //plt.fpoint(datax[i]/scalex,datay[i]/scaley);

        plt.fmove(0,offset+(double)i/10);
        sprintf(yMarker, " y = %.2f ", (double)(scaley*i/10));
        plt.label(yMarker);


    }
return 0;
}*/


