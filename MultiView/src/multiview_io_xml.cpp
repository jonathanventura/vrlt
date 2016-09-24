/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: multiview_io_xml.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <MultiView/multiview_io_xml.h>

#include <TinyXML/tinyxml.h>

namespace vrlt {
	namespace XML {
    
		static void readNodes( Reconstruction *r, TiXmlNode *root, ElementList &nodelist, Node *parent = NULL )
		{
			TiXmlNode *node;
			
			node = NULL;
			while ( (node = root->IterateChildren("node",node)) )
			{
				TiXmlElement *elem = node->ToElement();
				
				Node *mynode = new Node;
				mynode->parent = parent;
				
				const char *text;
				
				text = elem->Attribute("name");
				if ( text != NULL ) {
					mynode->name = std::string(text);
				}
					
				text = elem->Attribute("rotation");
				if ( text != NULL ) {
					double val[3];
					sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
					mynode->pose.so3() = Sophus::SO3d::exp( Eigen::Map<Eigen::Vector3d>( val ) );
				}
				
				text = elem->Attribute("translation");
				if ( text != NULL ) {
					double val[3];
					sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
					mynode->pose.translation() = Eigen::Map<Eigen::Vector3d>( val );
				}
				
				text = elem->Attribute("center");
				if ( text != NULL ) {
					double val[3];
					sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
					mynode->pose.translation() = - ( mynode->pose.so3() * Eigen::Map<Eigen::Vector3d>( val ) );
				}
				
				text = elem->Attribute("image");
				if ( text != NULL ) {
					mynode->camera = (Camera*) r->cameras[std::string(text)];
					mynode->camera->node = mynode;
				} else {
					mynode->camera = NULL;
				}
				
				text = elem->Attribute("fixed");
				if ( text != NULL ) {
					int val;
					sscanf( text, "%d", &val );
					if ( val != 0 ) mynode->fixed = true;
				}
				
				text = elem->Attribute("scaleFixed");
				if ( text != NULL ) {
					int val;
					sscanf( text, "%d", &val );
					if ( val != 0 ) mynode->scaleFixed = true;
				}
				
				TiXmlNode *child = NULL;
                while ( (child = node->IterateChildren("plane",child)) )
                {
                    TiXmlElement *childelem = child->ToElement();
					
					Plane *plane = new Plane;
                    
                    text = childelem->Attribute("name");
					if ( text != NULL ) {
						plane->name = std::string(text);
					}

					text = childelem->Attribute("normal");
					if ( text != NULL ) {
						double val[3];
						sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
						plane->normal = Eigen::Map<Eigen::Vector3d>( val );
					}
                    
                    text = childelem->Attribute("origin");
					if ( text != NULL ) {
						double val[3];
						sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
						plane->origin = Eigen::Map<Eigen::Vector3d>( val );
					}

                    text = childelem->Attribute("X");
					if ( text != NULL ) {
						double val[3];
						sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
						plane->X = Eigen::Map<Eigen::Vector3d>( val );
					}

                    text = childelem->Attribute("Y");
					if ( text != NULL ) {
						double val[3];
						sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
						plane->Y = Eigen::Map<Eigen::Vector3d>( val );
					}

                    mynode->planes[plane->name] = plane;
                    plane->node = mynode;
                }
                
                child = NULL;
				while ( (child = node->IterateChildren("point",child)) )
				{
					TiXmlElement *childelem = child->ToElement();
					
					Point *point = new Point;
					
					text = childelem->Attribute("name");
					if ( text != NULL ) {
						point->name = std::string(text);
					}
					
					text = childelem->Attribute("position");
					if ( text != NULL ) {
						double val[4];
						sscanf( text, "%lf %lf %lf %lf", &val[0], &val[1], &val[2], &val[3] );
						point->position = Eigen::Map<Eigen::Vector4d>( val );
					}
					
					text = childelem->Attribute("normal");
					if ( text != NULL ) {
						double val[3];
						sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
						point->normal = Eigen::Map<Eigen::Vector3d>( val );
					}
					
					text = childelem->Attribute("track");
					if ( text != NULL ) {
						point->track = (Track*)r->tracks[std::string(text)];
						point->track->point = point;
					}
                    
                    text = childelem->Attribute("plane");
					if ( text != NULL ) {
						point->plane = (Plane*)mynode->planes[std::string(text)];
					}
					
					mynode->points[point->name] = point;
					point->node = mynode;
				}

				readNodes( r, node, mynode->children, mynode );
				
				nodelist[mynode->name] = mynode;
			}
		}
		
		bool read( Reconstruction &r, const std::string &path, bool read_features )
		{
			TiXmlDocument doc( path.c_str() );
			
			doc.LoadFile();
			
			TiXmlNode *root = doc.RootElement();
			
			const char *text;
			text = root->ToElement()->Attribute("upright");
			if ( text != NULL ) {
				int val;
				sscanf(text,"%d",&val);
				r.upright = ( val != 0 );
			}

			text = root->ToElement()->Attribute("featureFileSuffix");
			if ( text != NULL ) {
				r.featureFileSuffix = std::string(text);
			}
            
            text = root->ToElement()->Attribute("utmZone");
            if ( text != NULL ) {
                r.utmZone = atoi(text);
            }
            
            text = root->ToElement()->Attribute("utmNorth");
            if ( text != NULL ) {
                int val;
                sscanf(text,"%d",&val);
                r.utmNorth = ( val != 0 );
            }
            
            text = root->ToElement()->Attribute("utmCenterEast");
            if ( text != NULL ) {
                double val;
                sscanf(text,"%lf",&val);
                r.utmCenterEast = val;
            }

            text = root->ToElement()->Attribute("utmCenterNorth");
            if ( text != NULL ) {
                double val;
                sscanf(text,"%lf",&val);
                r.utmCenterNorth = val;
            }
            
			TiXmlNode *node;
			
			node = NULL;
			while ( (node = root->IterateChildren("calibration",node)) )
			{
				TiXmlElement *elem = node->ToElement(); 
				
				const char *text;
				
				Calibration *calibration = new Calibration;
				
				text = elem->Attribute("name");
				if ( text != NULL ) {
					calibration->name = std::string(text);
				}
				
				text = elem->Attribute("type");
				if ( text != NULL ) {
					if ( strcasecmp( text, "perspective" ) == 0 ) calibration->type = Calibration::Perspective;
					if ( strcasecmp( text, "spherical" ) == 0 ) calibration->type = Calibration::Spherical;
				}
				
				text = elem->GetText();
				if ( text != NULL ) {
					sscanf( text, "%lf %lf %lf %lf %lf", &calibration->focal, &calibration->center[0], &calibration->center[1], &calibration->k1, &calibration->k2 );
				}
				
				r.calibrations[calibration->name] = calibration;
			}

			node = NULL;
			while ( (node = root->IterateChildren("image",node)) )
			{
				TiXmlElement *elem = node->ToElement();
				
				const char *text;
				
				Camera *camera = new Camera;
				
				text = elem->Attribute("name");
				if ( text != NULL ) {
					camera->name = std::string(text);
				}
				
				text = elem->Attribute("calibration");
				if ( text != NULL ) {
					camera->calibration = (Calibration*)r.calibrations[std::string(text)];
				}
				
				text = elem->GetText();
				if ( text != NULL ) {
					camera->path = std::string(text);
				}
				
				TiXmlNode *child = NULL;
				while ( (child = node->IterateChildren( child ) ) )
				{
					TiXmlElement *childelem = child->ToElement();
					if ( childelem == NULL ) continue;
					
					text = childelem->GetText();
					if ( text != NULL ) {
						if ( strcasecmp( child->Value(), "path" ) == 0 ) {
							camera->path = std::string(text);
						} else if ( strcasecmp( child->Value(), "attitude" ) == 0 ) {
							double attitude[9];
							sscanf( text, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", attitude+0, attitude+1, attitude+2, attitude+3, attitude+4, attitude+5, attitude+6, attitude+7, attitude+8 );
							camera->attitude = Sophus::SO3d( Eigen::Map< Eigen::Matrix<double,3,3,Eigen::RowMajor> >(attitude) );
						} else if ( strcasecmp( child->Value(), "heading" ) == 0 ) {
							double heading;
							sscanf( text, "%lf", &heading );
							camera->heading = heading;
						} else if ( strcasecmp( child->Value(), "timestamp" ) == 0 ) {
							double timestamp;
							sscanf( text, "%lf", &timestamp );
							camera->timestamp = timestamp;
						} else if ( strcasecmp( child->Value(), "rotationRate" ) == 0 ) {
							double rate[3];
							sscanf( text, "%lf %lf %lf", rate+0, rate+1, rate+2 );
							camera->rotationRate = Eigen::Map<Eigen::Vector3d>( rate );
						}
					}
				}
				
				r.cameras[camera->name] = camera;
				
				if ( read_features ) readFeatures( r, camera );
			}
			
			node = NULL;
			while ( (node = root->IterateChildren("match",node)) )
			{
				TiXmlElement *elem = node->ToElement();
				
				const char *text;
				
				Match *match = new Match;
				
				text = elem->Attribute("name");
				if ( text != NULL ) {
					match->name = std::string(text);
				}

				text = elem->Attribute("score");
				if ( text != NULL ) {
					double val = 0;
					sscanf( text, "%lf", &val );
					match->score = val;
				}
				
				TiXmlNode *childnode = NULL;
				
				int count = 0;
				while ( (childnode = node->IterateChildren("featureref",childnode)) )
				{
					TiXmlElement *child = childnode->ToElement();
					text = child->Attribute("feature");
					if ( count++ == 0 ) {
						match->feature1 = (Feature*)r.features[std::string(text)];
						match->feature1->matches[ match->name ] = match;
					} else {
						match->feature2 = (Feature*)r.features[std::string(text)];
						match->feature2->matches[ match->name ] = match;
					}
				}
				
				r.matches[match->name] = match;
			}

			if ( read_features ) {
				node = NULL;
				while ( (node = root->IterateChildren("track",node)) )
				{
					TiXmlElement *elem = node->ToElement();
					
					const char *text;
					
					Track *track = new Track;
					
					text = elem->Attribute("name");
					if ( text != NULL ) {
						track->name = std::string(text);
					}
					
					TiXmlNode *childnode = NULL;
					
					while ( (childnode = node->IterateChildren("featureref",childnode)) )
					{
						TiXmlElement *child = childnode->ToElement();
						text = child->Attribute("feature");
						Feature *feature = (Feature*)r.features[std::string(text)];
						track->features[feature->name] = feature;
						feature->track = track;
					}
					
					r.tracks[track->name] = track;
				}
			}
			
			readNodes( &r, root, r.nodes );

			node = NULL;
			while ( (node = root->IterateChildren("pair",node)) )
			{
				TiXmlElement *elem = node->ToElement();
				
				const char *text;
				
				Pair *nodepair = new Pair;
				
				text = elem->Attribute("name");
				if ( text != NULL ) {
					nodepair->name = std::string(text);
				}
				
				text = elem->Attribute("node1");
				if ( text != NULL ) {
					nodepair->node1 = (Node*)r.nodes[std::string(text)];
				}
				
				text = elem->Attribute("node2");
				if ( text != NULL ) {
					nodepair->node2 = (Node*)r.nodes[std::string(text)];
				}
				
				text = elem->Attribute("rotation");
				if ( text != NULL ) {
					double val[3];
					sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
					nodepair->pose.so3() = Sophus::SO3d::exp( Eigen::Map<Eigen::Vector3d>( val ) );
				}
				
				text = elem->Attribute("translation");
				if ( text != NULL ) {
					double val[3];
					sscanf( text, "%lf %lf %lf", &val[0], &val[1], &val[2] );
					nodepair->pose.translation() = Eigen::Map<Eigen::Vector3d>( val );
				}
				
				text = elem->Attribute("nmatches");
				if ( text != NULL ) {
					int val;
					sscanf( text, "%d", &val );
					nodepair->nmatches = val;
				}
				
				r.pairs[nodepair->name] = nodepair;
			}
			
			return true;
		}
		
		static void writeNodes( TiXmlElement *root, ElementList &nodelist )
		{
			ElementList::iterator it;
			for ( it = nodelist.begin(); it != nodelist.end(); it++ ) {
				Node *node = (Node*) it->second;

				TiXmlElement *elem = new TiXmlElement( "node" );
				
				elem->SetAttribute( "name", node->name.c_str() );
				
				char text[1024];
				
                Eigen::Vector3d vec;
				
				vec = node->pose.so3().log();
				sprintf( text, "%lf %lf %lf", vec[0], vec[1], vec[2] );
				elem->SetAttribute( "rotation", text );
				
				vec = node->pose.translation();
				sprintf( text, "%lf %lf %lf", vec[0], vec[1], vec[2] );
				elem->SetAttribute( "translation", text );
				
				if ( node->camera != NULL ) {
					elem->SetAttribute( "image", node->camera->name.c_str() );
				}
				
				if ( node->fixed )
				{
					elem->SetAttribute( "fixed", "1" );
				}

				if ( node->scaleFixed )
				{
					elem->SetAttribute( "scaleFixed", "1" );
				}
                
                ElementList::iterator planeit;
				for ( planeit = node->planes.begin(); planeit != node->planes.end(); planeit++ )
				{
					Plane *plane = (Plane *)planeit->second;
					
					TiXmlElement *childelem = new TiXmlElement( "plane" );
                    
					childelem->SetAttribute( "name", plane->name.c_str() );
					
					sprintf( text, "%lf %lf %lf", plane->normal[0], plane->normal[1], plane->normal[2] );
					childelem->SetAttribute( "normal", text );
                    
					sprintf( text, "%lf %lf %lf", plane->origin[0], plane->origin[1], plane->origin[2] );
					childelem->SetAttribute( "origin", text );

                    sprintf( text, "%lf %lf %lf", plane->X[0], plane->X[1], plane->X[2] );
					childelem->SetAttribute( "X", text );

                    sprintf( text, "%lf %lf %lf", plane->Y[0], plane->Y[1], plane->Y[2] );
					childelem->SetAttribute( "Y", text );

					elem->LinkEndChild( childelem );
				}
				
				ElementList::iterator pointit;
				for ( pointit = node->points.begin(); pointit != node->points.end(); pointit++ )
				{
					Point *point = (Point *)pointit->second;
					
					TiXmlElement *childelem = new TiXmlElement( "point" );
				   
					childelem->SetAttribute( "name", point->name.c_str() );
					
					sprintf( text, "%lf %lf %lf %lf", point->position[0], point->position[1], point->position[2], point->position[3] );
					childelem->SetAttribute( "position", text );

					sprintf( text, "%lf %lf %lf", point->normal[0], point->normal[1], point->normal[2] );
					childelem->SetAttribute( "normal", text );

					if ( point->track != NULL )
					{
						childelem->SetAttribute( "track", point->track->name.c_str() );
					}
                    
                    if ( point->plane != NULL )
                    {
                        childelem->SetAttribute( "plane", point->plane->name.c_str() );
                    }
					
					elem->LinkEndChild( childelem );
				}
				
				writeNodes( elem, node->children );
				
				root->LinkEndChild( elem );
			}
		}
		
		bool write( Reconstruction &r, const std::string &path )
		{
			TiXmlDocument doc;
			
			TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
			doc.LinkEndChild( decl );
			
			TiXmlElement *root = new TiXmlElement( "reconstruction" );
			
			char text[256];
			sprintf( text, "%d", (r.upright) ? 1 : 0 );
			root->SetAttribute( "upright", text );
			
			root->SetAttribute( "featureFileSuffix", r.featureFileSuffix.c_str() );
			
            if ( r.utmZone != 0 )
            {
                sprintf( text, "%d", r.utmZone );
                root->SetAttribute( "utmZone", text );
                
                sprintf( text, "%d", (r.utmNorth) ? 1 : 0 );
                root->SetAttribute( "utmNorth", text );
                
                sprintf( text, "%0.17lf", r.utmCenterEast );
                root->SetAttribute( "utmCenterEast", text );

                sprintf( text, "%0.17lf", r.utmCenterNorth );
                root->SetAttribute( "utmCenterNorth", text );
            }
            
			ElementList::iterator it;
			
			for ( it = r.calibrations.begin(); it != r.calibrations.end(); it++ ) {
				Calibration *calibration = (Calibration*) it->second;
				
				TiXmlElement *elem = new TiXmlElement( "calibration" );
				
				elem->SetAttribute( "name", calibration->name.c_str() );
				
				switch ( calibration->type )
				{
					case Calibration::Spherical:
						elem->SetAttribute( "type", "spherical" );
						break;
						
					default:
						break;
				}
				
				char textdata[256];
				sprintf( textdata, "%lf %lf %lf %lf %lf", calibration->focal, calibration->center[0], calibration->center[1], calibration->k1, calibration->k2 );
				TiXmlText *text = new TiXmlText( textdata );
				elem->LinkEndChild( text );
				
				root->LinkEndChild( elem );
			}
			
			for ( it = r.cameras.begin(); it != r.cameras.end(); it++ ) {
				Camera *camera = (Camera*) it->second;
				
				TiXmlElement *elem = new TiXmlElement( "image" );
				
				elem->SetAttribute( "name", camera->name.c_str() );
				elem->SetAttribute( "calibration", camera->calibration->name.c_str() );

				TiXmlElement *child = NULL;
				TiXmlText *text = NULL;
				char vals[1024];
				
				if ( camera->attitude.log().norm() != 0. ) {
					child = new TiXmlElement( "attitude" );
                    Eigen::Matrix3d rot = camera->attitude.matrix();
					sprintf( vals, "%.15lf %.15lf %.15lf    %.15lf %.15lf %.15lf    %.15lf %.15lf %.15lf", rot(0,0), rot(0,1), rot(0,2),    rot(1,0), rot(1,1), rot(1,2),   rot(2,0), rot(2,1), rot(2,2) );
					text = new TiXmlText( vals );
					child->LinkEndChild( text );
					elem->LinkEndChild( child );
				}
				
				child = new TiXmlElement( "path" );
				text = new TiXmlText( camera->path.c_str() );
				child->LinkEndChild( text );
				elem->LinkEndChild( child );
				
				if ( camera->timestamp != 0 ) {
					child = new TiXmlElement( "timestamp" );
					sprintf( vals, "%.15lf", camera->timestamp );
					text = new TiXmlText( vals );
					child->LinkEndChild( text );
					elem->LinkEndChild( child );
				}
				
				if ( camera->rotationRate.norm() != 0 ) {
					child = new TiXmlElement( "rotationRate" );
                    Eigen::Vector3d rate = camera->rotationRate;
					sprintf( vals, "%.15lf %.15lf %.15lf", rate[0], rate[1], rate[2] );
					text = new TiXmlText( vals );
					child->LinkEndChild( text );
					elem->LinkEndChild( child );
				}
				
				root->LinkEndChild( elem );
			}

			for ( it = r.matches.begin(); it != r.matches.end(); it++ ) {
				Match *match = (Match*) it->second;
				
				TiXmlElement *elem = new TiXmlElement( "match" );
				
				elem->SetAttribute( "name", match->name.c_str() );
				
				char text[256];
				sprintf( text, "%lf", match->score );
				elem->SetAttribute( "score", text );
				
				TiXmlElement *child;
				
				child = new TiXmlElement( "featureref" );
				child->SetAttribute( "feature", match->feature1->name.c_str() );
				elem->LinkEndChild( child );
				
				child = new TiXmlElement( "featureref" );
				child->SetAttribute( "feature", match->feature2->name.c_str() );
				elem->LinkEndChild( child );
				
				root->LinkEndChild( elem );
			}
			
			for ( it = r.tracks.begin(); it != r.tracks.end(); it++ ) {
				Track *track = (Track*) it->second;
				
				TiXmlElement *elem = new TiXmlElement( "track" );
				
				elem->SetAttribute( "name", track->name.c_str() );
				
				TiXmlElement *child;
				
				ElementList::iterator childit;
				for ( childit = track->features.begin(); childit != track->features.end(); childit++ )
				{
					Feature *feature = (Feature*) childit->second;
					
					child = new TiXmlElement( "featureref" );
					child->SetAttribute( "feature", feature->name.c_str() );
					elem->LinkEndChild( child );
				}
				
				root->LinkEndChild( elem );
			}
			
			writeNodes( root, r.nodes );
			
			for ( it = r.pairs.begin(); it != r.pairs.end(); it++ ) {
				Pair *nodepair = (Pair*) it->second;
				
				TiXmlElement *elem = new TiXmlElement( "pair" );
				
				elem->SetAttribute( "name", nodepair->name.c_str() );
				elem->SetAttribute( "node1", nodepair->node1->name.c_str() );
				elem->SetAttribute( "node2", nodepair->node2->name.c_str() );
				
				char text[256];
				sprintf( text, "%d", nodepair->nmatches );
				elem->SetAttribute( "nmatches", text );
				
                Eigen::Vector3d vec;
				
				vec = nodepair->pose.so3().log();
				sprintf( text, "%lf %lf %lf", vec[0], vec[1], vec[2] );
				elem->SetAttribute( "rotation", text );
				
				vec = nodepair->pose.translation();
				sprintf( text, "%lf %lf %lf", vec[0], vec[1], vec[2] );
				elem->SetAttribute( "translation", text );

				root->LinkEndChild( elem );
			}
			
			doc.LinkEndChild( root );

			doc.SaveFile( path.c_str() );
			
			return true;
		}

		void readFeatures( Reconstruction &r, Camera *camera )
		{
			char filename[256];
			sprintf( filename, "%s/%s.%s", r.pathPrefix.c_str(), camera->path.c_str(), r.featureFileSuffix.c_str() );
			
			FILE *f = fopen( filename, "r" );
			if ( f == NULL ) return;
			
			while ( true )
			{
				int namelength = 0;
				fread( &namelength, sizeof(int), 1, f );
				if ( namelength == 0 ) break;
				char *name = new char[namelength+1];
				fread( name, 1, namelength, f );
				name[namelength] = '\0';
				Feature *feature = new Feature;
				feature->name = name;
				feature->camera = camera;
				double vals[4];
				fread( vals, sizeof(double), 4, f );
				feature->location[0] = vals[0];
				feature->location[1] = vals[1];
				feature->orientation = vals[2];
				feature->scale = vals[3];
				fread( feature->color, 1, 3, f );
				camera->features[ feature->name ] = feature;
				r.features[ feature->name ] = feature;
				delete[] name;
			}
			
			fclose( f );
		}

		void writeFeatures( Reconstruction &r, Camera *camera )
		{
			char filename[256];
			sprintf( filename, "%s/%s.%s", r.pathPrefix.c_str(), camera->path.c_str(), r.featureFileSuffix.c_str() );
			
			FILE *f = fopen( filename, "w" );
			
			ElementList::iterator it;
			for ( it = camera->features.begin(); it != camera->features.end(); it++ )
			{
				Feature *feature = (Feature*)it->second;
				int namelength = (int)feature->name.size();
				fwrite( &namelength, sizeof(int), 1, f );
				fwrite( feature->name.c_str(), 1, namelength, f );
				double vals[4];
				vals[0] = feature->location[0];
				vals[1] = feature->location[1];
				vals[2] = feature->orientation;
				vals[3] = feature->scale;
				fwrite( vals, sizeof(double), 4, f );
				fwrite( feature->color, 1, 3, f );
			}
			
			fclose( f );
		}
		
		void readDescriptors( Reconstruction &r, Camera *camera )
		{
			char filename[256];
			sprintf( filename, "%s/%s.%s.dat", r.pathPrefix.c_str(), camera->path.c_str(), r.featureFileSuffix.c_str() );
			
			FILE *f = fopen( filename, "r" );
			
			while ( true )
			{
				int namelength = 0;
				fread( &namelength, sizeof(int), 1, f );
				if ( namelength == 0 ) break;
				char *name = new char[namelength+1];
				fread( name, 1, namelength, f );
				name[namelength] = '\0';
				Feature *feature = (Feature*)camera->features[std::string(name)];
				feature->descriptor = new unsigned char[128];
				fread( feature->descriptor, 1, 128, f );
				delete[] name;
			}
			
			fclose( f );
		}
		
		void clearDescriptors( Reconstruction &r, Camera *camera )
		{
			ElementList::iterator it;
			for ( it = camera->features.begin(); it != camera->features.end(); it++ )
			{
				Feature *feature = (Feature*)it->second;
				delete feature->descriptor;
				feature->descriptor = NULL;
			}
		}
		
		void writeDescriptors( Reconstruction &r, Camera *camera )
		{
			char filename[256];
			sprintf( filename, "%s/%s.%s.dat", r.pathPrefix.c_str(), camera->path.c_str(), r.featureFileSuffix.c_str() );
			
			FILE *f = fopen( filename, "w" );
			
			ElementList::iterator it;
			for ( it = camera->features.begin(); it != camera->features.end(); it++ )
			{
				Feature *feature = (Feature*)it->second;
				int namelength = (int)feature->name.size();
				fwrite( &namelength, sizeof(int), 1, f );
				fwrite( feature->name.c_str(), 1, namelength, f );
				fwrite( feature->descriptor, 1, 128, f );
			}
			
			fclose( f );
		}
		
		void readDescriptors( Reconstruction &r, Node *node )
		{
			if ( node->camera != NULL )
			{
				readDescriptors( r, node->camera );
			}
			
			ElementList::iterator it;
			for ( it = node->children.begin(); it != node->children.end(); it++ )
			{
				Node *child = (Node *)it->second;
				readDescriptors( r, child );
			}
		}
		
		void clearDescriptors( Reconstruction &r, Node *node )
		{
			if ( node->camera != NULL )
			{
				clearDescriptors( r, node->camera );
			}
			
			ElementList::iterator it;
			for ( it = node->children.begin(); it != node->children.end(); it++ )
			{
				Node *child = (Node *)it->second;
				clearDescriptors( r, child );
			}
		}
	}
}
