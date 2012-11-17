/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: multiview_io_xml.h
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#ifndef __MULTIVIEW_IO_XML_H
#define __MULTIVIEW_IO_XML_H

#include <MultiView/multiview.h>

namespace MultiView {
	namespace XML {
/** \addtogroup MultiViewXMLIO MultiView XML I/O
 * \brief Reading and writing of MultiView data to an XML file.
 * @{
 */
        
		bool read( Reconstruction &r, const std::string &path, bool read_features = true );
		bool write( Reconstruction &r, const std::string &path );
        
		void readFeatures( Reconstruction &r, Camera *camera );
		void writeFeatures( Reconstruction &r, Camera *camera );
	
		void readDescriptors( Reconstruction &r, Camera *camera );
		void clearDescriptors( Reconstruction &r, Camera *camera );
		void writeDescriptors( Reconstruction &r, Camera *camera );

		void readDescriptors( Reconstruction &r, Node *node );
		void clearDescriptors( Reconstruction &r, Node *node );
        
/**
 * @}
 */
	}
}

#endif
