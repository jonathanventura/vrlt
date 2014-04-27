/*
 * Copyright (c) 2014. Jonathan Ventura. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: osm_xml_reader.h
 * Author: Jonathan Ventura
 * Last Modified: 04.27.2014
 */

#ifndef __OSM_XML_H
#define __OSM_XML_H

#include <vector>
#include <map>
#include <string>

namespace vrlt {
    struct OSMNode
    {
        size_t ID;
        double lat;
        double lon;
        double east;
        double north;
    };
    typedef std::map<size_t,OSMNode*> OSMNodeList;
    
    struct OSMWay
    {
        size_t ID;
        bool building;
        std::vector<size_t> nodeids;
    };
    typedef std::map<size_t,OSMWay*> OSMWayList;

    class OSMData
    {
    public:
        int utm_zone;
        bool utm_north;
        
        OSMNodeList nodes;
        OSMWayList ways;
        
        bool read( const std::string &path );

        ~OSMData();
    };
}

#endif
