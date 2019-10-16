/*
** Heriott Watt University, 2019
** Intelligent Robotics
** Authors: Dale Carter, Rohit Chakraborty, Lucas Dumy, Solène Navaro, Emanuele De Pellegrin
*/

#include "ImageAnalysis.hpp"

int main()
{
    ImageAnalysis analyser;

    if (analyser.initImageAnalysis()) {
        printf("The program couldn't be initialised due to an internal error.\n");
        return (-1);
    }
    return (analyser.runAnalysis());
}