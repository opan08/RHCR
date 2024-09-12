#pragma once
#include "BasicGraph.h"


class SortingGrid :
	public BasicGraph
{
public:
    unordered_map<std::string, int> inducts;//保存入库点的名称以及点位所在栅格的序号，first为名称，second为induct的位置
    unordered_map<string, list<int> > ejects; //保存出库点的名称以及出库点的fiducial，同个名称的出库点可能有多个点位 one eject station could have multiple eject fiducials 

    bool load_map(string fname);
    void preprocessing(bool consider_rotation); // compute heuristics
};
