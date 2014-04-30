/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: RenderTrack.cpp
 * Author: Jonathan Ventura
 * Last Modified: 25.2.2013
 */

#include <MultiView/multiview.h>
#include <MultiView/multiview_io_xml.h>

#include <GLUtils/GLModel.h>
#include <GLUtils/LoadOBJ.h>
#include <GLUtils/GLShadow.h>
#include <GLUtils/GLSLHelpers.h>
#include <GLUtils/GLSLPrograms.h>

#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>

#include <iostream>

using namespace vrlt;

GLModelShader modelShader;
GLModel *model = NULL;

GLShadowShader shadowShader;

GLImageShader imageShader;
GLGenericDrawable imageDrawable;

GLPointShader pointShader;
GLGenericDrawable pointDrawable;
GLGenericDrawable annotationsDrawable;
GLGenericDrawable gridDrawable;

Reconstruction r;
Reconstruction query;
ElementList::iterator queryit;

Node *annotations = NULL;

GLuint texID;

bool tracked = false;
bool showPoints = false;
bool showModel = true;
bool showGrid = false;
bool paused = false;

bool recordingOn = false;

int width, height;
Eigen::Matrix4d scale;
Eigen::Matrix4d proj;
Eigen::Vector3d lightDirection;
Eigen::Matrix4d modelRotation;
Eigen::Matrix4d modelOffset;
Eigen::Matrix4d modelView;
Eigen::Matrix4d modelViewProj;
Eigen::Matrix4d modelScale;
Eigen::Vector4d plane;

bool haveModel = false;
std::vector< Eigen::Matrix4d > modelOffsets;
std::vector< Eigen::Matrix4d > modelRotations;

bool wroteFrame = false;

static const float levelscale = 1;

static const int gridradius = 100;
static const int gridstep = 10;

void saveImage()
{
    static int mycounter = 1;
    char path[256];
    sprintf( path, "Output/frame%04d.jpg", mycounter++ );
    cv::Mat image( cv::Size( width, height ), CV_8UC3 );
    glReadPixels( 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image.data );
    cv::flip( image, image, 0 );
    cv::imwrite( path, image );
}

void renderBitmapString( float x, float y, void *font, const char *string ) {
    const char *c;
    glRasterPos2f(x, y);
    for (c=string; *c != '\0'; c++)
    {
        glutBitmapCharacter(font, *c);
    }
}

void drawText( const char* string, float xpos, float ypos )
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1, 1 );
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glColor3f(0, 0, 1.0);
    //renderBitmapString( xpos, ypos, GLUT_BITMAP_9_BY_15, string );
    glPointSize( 100. );
    glBegin( GL_POINTS );
    glVertex2f( xpos, ypos );
    glEnd();
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void display()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    
    glBindTexture( GL_TEXTURE_2D, texID );
    imageDrawable.PushClientState();
    imageShader.shaderProgram.Use();
    imageDrawable.Draw( GL_TRIANGLE_STRIP );
    imageDrawable.PopClientState();
    
    if ( tracked ) {
        if ( showPoints ) {
            pointShader.shaderProgram.Use();
            Eigen::Vector3d color;
            color << 1, 0, 0;
            pointShader.SetColor( color );
            pointDrawable.PushClientState();
            pointDrawable.Draw( GL_POINTS );
            pointDrawable.PopClientState();
        }
        
        if ( annotations != NULL )
        {
            pointShader.shaderProgram.Use();
            Eigen::Vector3d color;
            color << 0, 1, 0;
            pointShader.SetColor( color );
            annotationsDrawable.PushClientState();
            annotationsDrawable.Draw( GL_POINTS );
            annotationsDrawable.PopClientState();
        }
        
        if ( showGrid ) {
            pointShader.shaderProgram.Use();
            Eigen::Vector3d color;
            color << 1, 1, 0;
            pointShader.SetColor( color );
            gridDrawable.PushClientState();
            gridDrawable.Draw( GL_LINES );
            gridDrawable.PopClientState();
        }
        
//        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        if ( showModel && haveModel ) {
            glEnable( GL_DEPTH_TEST );
            model->drawable->PushClientState();
            
            for ( int i = 0; i < modelOffsets.size(); i++ ) {
                Eigen::Matrix4d my_modelViewProj = modelViewProj * modelOffsets[i] * modelRotations[i] * modelScale;
                modelShader.shaderProgram.Use();
                modelShader.SetModelViewProj( my_modelViewProj );
                model->Render();
                
                my_modelViewProj = modelViewProj * modelOffsets[i] * modelRotation * modelScale;
                glBindTexture( GL_TEXTURE_2D, texID );
                shadowShader.shaderProgram.Use();
                shadowShader.SetModelViewProj( my_modelViewProj );
                glEnable( GL_BLEND );
                model->Render();
                glDisable( GL_BLEND );
            }
            
            model->drawable->PopClientState();
            glDisable( GL_DEPTH_TEST );
        }
//        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        
    }
    
    glutSwapBuffers();

    if ( recordingOn ) {
        if ( !wroteFrame ) {
            saveImage();
            wroteFrame = true;
        }
    }

    glutPostRedisplay();
}

void keyboard( unsigned char key, int x, int y )
{
    switch ( key )
    {
        case ' ':
            paused = !paused;
            break;
            
        case 's':
        {
            static int snapshotCounter = 0;
            char path[256];
            sprintf( path, "Output/snapshot%04d.png", snapshotCounter++ );
            cv::Mat image( cv::Size( width, height ), CV_8UC3 );
            glReadPixels( 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image.data );
            cv::flip( image, image, 0 );
            cv::imwrite( path, image );
            break;
        }
            
        case 'p':
            showPoints = !showPoints;
            break;
            
        case 'm':
            showModel = !showModel;
            break;
            
        case 'g':
            showGrid = !showGrid;
            break;
            
        case 'q':
            exit(0);
            break;
            
        case 'r':
            recordingOn = !recordingOn;
            break;
            
        default:
            break;
    }
}

void setPlanePoint( int x, int y )
{
    Camera *camera = (Camera*)queryit->second;
    Node *node = camera->node;
    
    if ( node == NULL ) return;
    if ( node->parent == NULL ) return;
    
    // intersect with ground plane
    Eigen::Vector4d ground_plane = plane;//makeVector( 0, -1, 0, 0 );
    
    Eigen::Vector2d screen_pos;
    screen_pos << x/levelscale, y/levelscale;
    
    Eigen::Vector3d origin = -( node->pose.so3().inverse() * node->pose.translation() );
    Eigen::Vector3d ray = node->pose.so3().inverse() * camera->calibration->unproject( screen_pos );
    
    // N * ( x0 + r * t ) + D = 0
    // N * x0 + N * r * t + D = 0
    // t = ( - D - N * x0 ) / ( N * r )
    
    double t = ( - ground_plane[3] - ground_plane.head(3).dot( origin ) ) / ( ground_plane.head(3).dot( ray ) );
    Eigen::Vector3d point = origin + t * ray;
    
    std::cout << "point: " << point << "\n";
    
    modelOffset = makeTranslation( point );
}

void remakeMatrices()
{
    Camera *camera = (Camera*)queryit->second;
    Node *node = camera->node;

    if ( node == NULL ) return;
    if ( node->parent == NULL ) return;
    
    Sophus::SE3d pose = node->globalPose();
    modelView = makeModelView( pose );
    modelViewProj = proj * scale * modelView;
    pointShader.shaderProgram.Use();
    pointShader.SetModelViewProj( modelViewProj );
//    
//    modelViewProj = modelViewProj * modelOffset * modelRotation * modelScale;
//    modelShader.shaderProgram.Use();
//    modelShader.SetModelViewProj( modelViewProj );
//    shadowShader.shaderProgram.Use();
//    shadowShader.SetModelViewProj( modelViewProj );
}

void mouse( int button, int state, int x, int y )
{
    if ( button == GLUT_LEFT_BUTTON ) {
        if ( state == GLUT_DOWN ) {
            setPlanePoint( x, y );
            modelOffsets.push_back( modelOffset );
            
//            modelRotations.push_back( makeRotation( SO3<>( makeVector( 0, rand_u()*2*M_PI, 0 ) ) ) );
            modelRotations.push_back( modelRotation );
//            remakeMatrices();
        }
    }
}

void motion( int x, int y )
{
    if ( modelOffsets.empty() ) return;
    
    setPlanePoint( x, y );
    modelOffsets.back() = modelOffset;
//    remakeMatrices();
}

void idle()
{
    if ( paused ) return;
    
    // need to add some code to keep this at 30 Hz
    
    queryit++;
    if ( queryit == query.cameras.end() ) {
        if ( recordingOn ) exit(0);
        else queryit = query.cameras.begin();
    }
    
    Camera *camera = (Camera*)queryit->second;
    Node *node = camera->node;
    
    cv::Mat im = cv::imread( camera->path, cv::IMREAD_COLOR );
    glBindTexture( GL_TEXTURE_2D, texID );
    glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, im.data );
    
    //    Camera *firstcamera = (Camera*)r.cameras.begin()->second;
    //    camera->node->pose = firstcamera->node->pose;
    
    tracked = false;
    if ( node != NULL ) {
        if ( node->parent != NULL ) {
            tracked = true;
            remakeMatrices();
        }
    }
    
    wroteFrame = false;
    
//    glutPostRedisplay();
    //paused = true;
}

void setupGL()
{
    glClearColor( 0.0, 0.0, 0.0, 0.0 );

    Camera *camera = (Camera*)queryit->second;
    cv::Mat im = cv::imread( camera->path, cv::IMREAD_COLOR );

    width = im.size().width;
    height = im.size().height;
    
    glViewport( 0, 0, width*levelscale, height*levelscale );

    Calibration *calibration = camera->calibration;
    
    Eigen::Vector4d params;
    params[0] = calibration->focal;
    params[1] = -calibration->focal;
    params[2] = calibration->center[0];
    params[3] = calibration->center[1];
    params[0] *= levelscale;
    params[1] *= levelscale;
    params[2] = ( params[2] + 0.5f ) * levelscale - 0.5f;
    params[3] = ( params[3] + 0.5f ) * levelscale - 0.5f;
    proj = makeProj( params, width*levelscale, height*levelscale, 0.1, 1000 );
    
    scale = Eigen::Matrix4d::Identity();
    scale(2,2) = -1;
    
    glGenTextures( 1, &texID );
    glBindTexture( GL_TEXTURE_2D, texID );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    
    imageShader.Create();
    imageShader.shaderProgram.Use();
    imageShader.SetTextureUnit( 0 );
    
    imageDrawable.Create();
    imageDrawable.AddAttrib( 0, 2 );
    imageDrawable.AddAttrib( 1, 2 );
    imageDrawable.AddElem( makeVector(-1.f,-1.f ), makeVector( 0.f, 1.f ) );
    imageDrawable.AddElem( makeVector( 1.f,-1.f ), makeVector( 1.f, 1.f ) );
    imageDrawable.AddElem( makeVector(-1.f, 1.f ), makeVector( 0.f, 0.f ) );
    imageDrawable.AddElem( makeVector( 1.f, 1.f ), makeVector( 1.f, 0.f ) );
    imageDrawable.Commit();

//    lightDirection = makeVector( -1, 1, 0.5 );
//    plane = makeVector( 0, -1, 0, 0 );
//    modelScale = makeScale( makeVector( 1, 1, 1 ) );
//    modelRotation = makeRotation( SO3<>( makeVector( 0, 0, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, 0, 0 ) );
//
//    modelOffsets.push_back( makeTranslation( makeVector( 0, 0, 0 ) ) );
//    modelRotations.push_back( modelRotation );

    // for logo (physicalsci)
//    lightDirection = makeVector( -1, 1, 1 );
//    plane = makeVector( 0, -1, 0, 0 );
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * .2 ); // inches to meters
//    modelRotation = makeRotation( SO3<>( makeVector( 0, M_PI/2, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, 0, 0 ) );

    // for pinetree
//    lightDirection = makeVector( 1, 1, -1 );
//    lightDirection = makeVector( 1, 1, 1 );
//    plane = makeVector( 0, -1, 0, 0 );
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * .75 ); // inches to meters
//    modelRotation = makeRotation( SO3<>( makeVector( 0, 0, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, 0, 0 ) );

    // for lamp
//    lightDirection = makeVector( 1, 1, -1 );
//    //    plane = makeVector( 0, -1, 0, 0 );
//    plane = makeVector( 1, 0, 0, 0 );
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * 0.5 ); // inches to meters
////    modelScale = makeScale( makeVector( 1, 1, 1 )  ); // inches to meters
//    //    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * 0.5 ); // inches to meters
//    modelRotation = makeRotation( SO3<>( makeVector( -M_PI/2., 0, 0 ) ) * SO3<>( makeVector( 0, 0, M_PI / 2. ) ) * SO3<>( makeVector( 0, -M_PI, 0 ) ) );
////    modelRotation = makeRotation( SO3<>() );//SO3<>( makeVector( 0, 0, -M_PI / 2. ) ) * SO3<>( makeVector( 0, -M_PI, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, 0, 0 ) );
//    
//    cout << ( SO3<>( makeVector( -M_PI/2., 0, 0 ) ) * SO3<>( makeVector( 0, 0, M_PI / 2. ) ) * SO3<>( makeVector( 0, -M_PI, 0 ) ) ).ln() << "\n";

    // for shuttle
    lightDirection = makeVector( -1., 1., 0.5 );
    plane = makeVector( 0., -1., 0., 0. );
//    plane = makeVector( 1, 0, 0, 0 );
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * 0.005 ); // inches to meters
    modelScale = makeScale( makeVector( 1., 1., 1. ) * 0.0254 * 0.2 ); // inches to meters
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * 0.5 ); // inches to meters
    modelRotation = makeRotation( Sophus::SO3d::exp( makeVector( 0., -M_PI, 0. ) ) );
//    modelRotation = makeRotation( SO3<>( makeVector( 0, 0, -M_PI / 2. ) ) * SO3<>( makeVector( 0, -M_PI, 0 ) ) );
    modelOffset = makeTranslation( Eigen::Vector3d::Zero() );

//    modelOffsets.push_back( makeTranslation( makeVector( 35.1645, 0, 0.0557605 ) ) );
//    modelRotations.push_back( modelRotation );

    // for arcde
//    lightDirection = makeVector( .5, -1, -.5 );
//    lightDirection = makeVector( 0, -1, 0 );
//    plane = makeVector( 0, -1, 0, 0 );
//    //    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * 0.005 ); // inches to meters
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 0.0254 * .5 ); // inches to meters
//    modelRotation = makeRotation( SO3<>( makeVector( M_PI, 0, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, 0, 0 ) );
//
////    modelOffsets.push_back( makeTranslation( makeVector( 0.705318, 0, 0.196767 ) ) );
//    modelOffsets.push_back( makeTranslation( makeVector( 2.65334, 0, 0.260111 ) ) );
//
//    modelRotations.push_back( modelRotation );
    
    // for shuttle
//    lightDirection = makeVector( 0, 1, 0 );
//    modelScale = makeScale( makeVector( 1, 1, 1 ) * 1 );
//    //modelScale = makeScale( makeVector( 2., 2., 2. ) );
//    modelRotation = makeRotation( SO3<>() );
//    modelOffset = makeTranslation ( makeVector( 0, 0, 20 ) );
////    modelOffset = makeTranslation( makeVector( 7.5, 0, -5. ) );
//    plane = makeVector( 0, -1, 0, 3 );
    
    // for hartley and bunny
//    lightDirection = makeVector( 0, -1, 1 );
//    modelScale = makeScale( makeVector( 5, 5, 5 ) );
//    modelRotation = makeRotation( SO3<>( makeVector( -M_PI/2, 0, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, -.4, 3.25 ) );
//    plane = makeVector( 0, 1, 0, 0 );

    // for magazine and bunny
//    lightDirection = makeVector( 0, -1, 1 );
//    modelScale = makeScale( makeVector( 20, 20, 20 ) );
//    modelRotation = makeRotation( SO3<>( makeVector( -M_PI/2, 0, 0 ) ) );
//    modelOffset = makeTranslation( makeVector( 0, -.4, 5. ) );
//    plane = makeVector( 0, 1, 0, 0 );

    modelShader.Create();
    modelShader.shaderProgram.Use();
    lightDirection.normalize();
    modelShader.SetLightDirection( lightDirection );
    modelShader.SetAmbient( .4 );
    
    shadowShader.Create( true );
    shadowShader.shaderProgram.Use();
    shadowShader.SetLightDirection( lightDirection );
    shadowShader.SetColor( makeVector( 0., 0., 0., .2 ) );
    shadowShader.SetPlane( plane );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    pointShader.Create();
    pointShader.shaderProgram.Use();
    pointShader.SetColor( makeVector( 1., 0., 0. ) );
    
    pointDrawable.Create();
    pointDrawable.AddAttrib( 0, 3 );
    ElementList::iterator it;
    Node *node = (Node*)r.nodes["root"];
    for ( it = node->points.begin(); it != node->points.end(); it++ )
    {
        Point *point = (Point *)it->second;
        
        pointDrawable.AddElem( project( point->position ).cast<float>() );
    }
    pointDrawable.Commit();
    
    annotationsDrawable.Create();
    annotationsDrawable.AddAttrib( 0, 3 );
    if ( annotations != NULL ) {
        for ( it = annotations->points.begin(); it != annotations->points.end(); it++ )
        {
            Point *point = (Point *)it->second;
            
            annotationsDrawable.AddElem( project( point->position ).cast<float>() );
        }
    }
    annotationsDrawable.Commit();
    
    gridDrawable.Create();
    gridDrawable.AddAttrib( 0, 3 );
    for ( int x = -gridradius; x <= gridradius; x += gridstep )
    {
        for ( int z = -gridradius; z < gridradius; z += gridstep )
        {
            gridDrawable.AddElem( makeVector( (float)x, 0.f, (float)z ) );
            gridDrawable.AddElem( makeVector( (float)x, 0.f, (float)(z+gridstep) ) );
        }
    }
    for ( int z = -gridradius; z <= gridradius; z += gridstep )
    {
        for ( int x = -gridradius; x < gridradius; x += gridstep )
        {
            gridDrawable.AddElem( makeVector( (float)x, 0.f, (float)z ) );
            gridDrawable.AddElem( makeVector( (float)(x+gridstep), 0.f, (float)z ) );
        }
    }
    gridDrawable.Commit();
    
    glPointSize( 5.0 );
}

int main( int argc, char **argv )
{
    if ( argc != 3 && argc != 5 ) {
        fprintf( stderr, "usage: %s <reconstruction> <tracked query> [<OBJ file prefix> <OBJ filename>]\n", argv[0] );
        return 0;
    }
    
    std::string pathin = std::string(argv[1]);
    std::string queryin = std::string(argv[2]);
    std::string objprefix;
    std::string objname;
    
    haveModel = false;
    if ( argc == 5 ) {
        haveModel = true;
        objprefix = std::string(argv[3]);
        objname = std::string(argv[4]);
    }
    
    r.pathPrefix = pathin;
    std::stringstream mypath;
    mypath << pathin << "/reconstruction.xml";
//    mypath << pathin << "/updated.xml";
    XML::read( r, mypath.str() );
    
    if ( r.nodes.count( "annotations" ) > 0 ) {
        annotations = (Node*)r.nodes["annotations"];
    }
    
    XML::read( query, queryin, false );
    queryit = query.cameras.begin();
    
    // set up GLUT
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
    glutInitWindowPosition( 20, 20 );
    glutInitWindowSize( 1280*levelscale, 720*levelscale );
    glutCreateWindow( "window" );
    
    model = NULL;
    if ( haveModel )
    {
        model = LoadOBJ( objprefix.c_str(), objname.c_str() );
    }
    setupGL();
    
    glutKeyboardFunc( keyboard );
    glutDisplayFunc( display );
    glutIdleFunc( idle );
    glutMouseFunc( mouse );
    glutMotionFunc( motion );
    
    glutMainLoop();
    
    return 0;
}
