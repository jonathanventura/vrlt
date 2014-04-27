/*
 * Copyright (c) 2014. Jonathan Ventura. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: osm_xml_reader.cpp
 * Author: Jonathan Ventura
 * Last Modified: 04.27.2014
 */

#include "osm_xml_reader.h"

#include <TinyXML/tinyxml.h>

#include <GeographicLib/UTMUPS.hpp>

namespace vrlt {
    bool OSMData::read( const std::string &path )
    {
        utm_zone = -1;
        
        TiXmlDocument doc( path.c_str() );
        
        doc.LoadFile();
        
        TiXmlNode *root = doc.RootElement();
        
        TiXmlNode *node;
        
        node = NULL;
        while ( (node = root->IterateChildren("node",node)) )
        {
            TiXmlElement *elem = node->ToElement(); 
            
            const char *text;
            
            OSMNode *osmnode = new OSMNode;
            
            text = elem->Attribute("id");
            if ( text != NULL ) {
                sscanf(text,"%lu",&osmnode->ID);
            } else {
                delete osmnode;
                continue;
            }
            
            text = elem->Attribute("lat");
            if ( text != NULL ) {
                sscanf(text,"%lf",&osmnode->lat);
            }
            
            text = elem->Attribute("lon");
            if ( text != NULL ) {
                sscanf(text,"%lf",&osmnode->lon);
            }
            
            double gamma;
            double k;
            GeographicLib::UTMUPS::Forward(osmnode->lat, osmnode->lon,
                                           this->utm_zone, this->utm_north,
                                           osmnode->east, osmnode->north,
                                           gamma, k );
            
            this->nodes[osmnode->ID] = osmnode;
        }

        node = NULL;
        while ( (node = root->IterateChildren("way",node)) )
        {
            TiXmlElement *elem = node->ToElement();
            
            const char *text;
            
            OSMWay *osmway = new OSMWay;
            osmway->building = false;
            
            text = elem->Attribute("id");
            if ( text != NULL ) {
                sscanf(text,"%lu",&osmway->ID);
            } else {
                delete osmway;
                continue;
            }
            
            TiXmlNode *child = NULL;
            while ( (child = node->IterateChildren( "nd", child ) ) )
            {
                TiXmlElement *childelem = child->ToElement();
                if ( childelem == NULL ) continue;
                
                text = childelem->Attribute("ref");
                if ( text != NULL ) {
                    size_t ID;
                    int nread = sscanf(text,"%lu",&ID);
                    if ( nread == 0 ) continue;
                    osmway->nodeids.push_back(ID);
                }
            }
            
            child = NULL;
            while ( (child = node->IterateChildren( "tag", child ) ) )
            {
                TiXmlElement *childelem = child->ToElement();
                if ( childelem == NULL ) continue;
                
                text = childelem->Attribute("k");
                if ( text != NULL ) {
                    if ( strcmp(text,"building")==0 ) osmway->building = true;
                }
            }

            
            this->ways[osmway->ID] = osmway;
        }
        
        return true;
    }
    
    OSMData::~OSMData()
    {
        for ( OSMNodeList::iterator it = nodes.begin(); it != nodes.end(); it++ )
        {
            delete it->second;
        }
        nodes.clear();
        
        for ( OSMWayList::iterator it = ways.begin(); it != ways.end(); it++ )
        {
            delete it->second;
        }
        ways.clear();
    }
}
