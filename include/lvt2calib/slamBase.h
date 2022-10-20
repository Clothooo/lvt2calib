/**************************************************************
    > File Name: SLAM_tutorial/part5/include/slamBase.h
    > Author: Zhang Jun
    > Mail: zjruyihust@gmail.com
    > Created Time: Sat, 14 Aug, 2017 10:40
    > Note: 高翔rgdb-slam教程所用到的基本函数（C 风格）
**************************************************************/
/* 
In the C and C++ programming languages, #pragma once is a non-standard 
but widely supported preprocessor directive designed to cause the current 
source file to be included only once in a single compilation. 
Thus, #pragma once serves the same purpose as include guards, but with 
several advantages, including: less code, avoidance of name clashes, 
and sometimes improvement in compilation d butspeed. 
    #ifndef GRANDPARENT_H
    #define GRANDPARENT_H
    ... contents of grandparent.h
    #endif // !GRANDPARENT_H 
*/

#pragma once    // Thus, #pragma once serves the same purpose as include guards, but with 
                // several advantages, including: ...

// Headers
// C++ standard libraries
#include <iostream>
#include <fstream>  // read file
#include <vector>
#include <map>

#include <cstdlib>
#include <stdio.h>      /* printf, fgets */
#include <stdlib.h>     /* atof */

// OpenCV
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

// parameter reader class
class ParameterReader
{
public:
    ParameterReader( string file):filename(file)    //    ParameterReader(e.g. string filename = "../parameters.txt" )
    //     This part can be moved to slamBase.cpp
    {
        //        filename = file;  // initialize parameters in construct function
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr << "parameter file does not exist." << endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );    // get fin, until '\n', then store to str.
            if (str[0] == '#' )
            {
                // comment starts with '#'
                continue;
            }

            int  pos = str.find(":");
            if (pos==-1)
                continue;
            string key = str.substr(0, pos);
            string value = str.substr(pos+2, str.length());
            data[key] = value;  // ? defined below in public:

            if(!fin.good())
                break;
        }
    }
    
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    } 

    //To read data from a text file.
    //filename is the name of the text file
    //rows and cols show dimensions of the matrix written in the text file
    Mat ReadMatFromTxt(string x, int rows,int cols)
    {
        char *str = const_cast<char  *>(x.c_str());
        char * pch;

        Mat out = Mat::zeros(rows, cols, CV_64FC1);     //Matrix to store values, e.g. printf ("Splitting string \"%s\" into tokens:\n",str);
        int cnt = 0;                                //index starts from 0
        pch = strtok (str, " ");
        // cout << "Type of *pch : " << typeid(*pch).name() << endl;   // type is c: char
        //        double p_d[length];

        while (pch != NULL)
        {
            int temprow = cnt / cols;
            int tempcol = cnt % cols;
            out.at<double>(temprow, tempcol) = atof(pch);   // convert char * to double
            //     printf ("p_d[%d] = %lf\n" ,cnt, p_d[cnt]); //  cout << p_d[0] << endl; // display is different
            cnt++;
            //            printf ("%s\n",pch);
            pch = strtok (NULL, " ");
        }
        return out;
    }

public:
    string filename;
    map<string, string> data;
};

