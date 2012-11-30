#include "image_map_visual.h"

#include "../include/map_visualization/definitions.h"
#include "mobots_common/constants.h"
#include "image_map_display.h"

namespace map_visualization
{

/**********************************************************
  * Constructor/Destructor
  ********************************************************/

/**
 * Creates the root node. Images are placed into a tree hierarchy.
 * root -> sessions -> mobots -> images
 */
ImageMapVisual::ImageMapVisual( Ogre::SceneManager* sceneManager_, ImageMapDisplay* display_){
	sceneManager = sceneManager_;
	// The root of the node tree containing all image nodes 
    rootImageNode = sceneManager->getRootSceneNode()->createChildSceneNode();
    rootMobotModelNode = sceneManager->getRootSceneNode()->createChildSceneNode();
    display = display_;
}

/**
 * All resources(nodes, textures, material) are deleted.
 */
ImageMapVisual::~ImageMapVisual()
{
    deleteAllImages();
    deleteAllMobotModels();
    sceneManager->destroySceneNode(rootImageNode);
    sceneManager->destroySceneNode(rootMobotModelNode);
}

/**********************************************************
  * Image Interfaces
  ********************************************************/

/*!
 Insert an image into Rviz and set the pose.
 All resources are uniquely named, enabling a clean deletion
 */
int ImageMapVisual::insertImage(int sessionID, int mobotID,	int imageID,
        float poseX, float poseY, float poseTheta, cv::Mat mat)
{
	// Get the node to which the image is assigned to
	Ogre::SceneNode* imageNode = getNode(sessionID, mobotID, imageID);
	// Deleting configuration and resources of a node with the same ID
	Ogre::SceneNode* parentNode = imageNode->getParentSceneNode();
	std::string imageNodeName = imageNode->getName();
	deleteImage(&imageNodeName);
    imageNode = parentNode->createChildSceneNode(imageNodeName,
		Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);

    //cv::namedWindow("recieved_image", 1);
    //cv::imshow("recieved_image", mat);
	
	// Create the material
	std::stringstream ss;
	ss << "MapMaterial-" << imageNode->getName();
	try{
        material = Ogre::MaterialManager::getSingleton().create(ss.str(),
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	} catch(Ogre::Exception& e) {
		Ogre::MaterialManager::getSingleton().remove(ss.str());
        material = Ogre::MaterialManager::getSingleton().create(ss.str(),
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	}
    material->setReceiveShadows(false);
    material->getTechnique(0)->setLightingEnabled(false);
    material->setDepthBias(-16.0f, 0.0f);
    material->setCullingMode(Ogre::CULL_NONE);
    material->setDepthWriteEnabled(false);
	
	// Create the texture
	std::stringstream ss2;
	ss2 << "MapTexture-" << imageNode->getName();
	try{
        texture = Ogre::TextureManager::getSingleton().createManual(
			ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, mat.cols, mat.rows, 0, Ogre::PF_X8R8G8B8,
			Ogre::TU_DEFAULT);
    } catch(Ogre::Exception& e) {
		Ogre::TextureManager::getSingleton().remove(ss2.str());
        texture = Ogre::TextureManager::getSingleton().createManual(
			ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, mat.cols, mat.rows, 0, Ogre::PF_X8R8G8B8,
			Ogre::TU_DEFAULT);
	}
	
	// Get pixel buffer
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();
	pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
	const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
	Ogre::uint32* pDest = static_cast<Ogre::uint32*>(pixelBox.data);
	
	// Fill in the pixel data.
	int k = 0;
	Ogre::uint32 bit32;
	for (int j = 0; j < mat.cols * mat.rows; j++){
		bit32 = ((mat.data[k]&0xFF)<<16) | ((mat.data[k + 1]&0xFF)<<8) | ((mat.data[k + 2]&0xFF));
		*pDest++ = bit32;
		k += 3;
	}
	pixelBuffer->unlock();
	
	// Get the current pass from the material
    Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
	if (pass->getNumTextureUnitStates() > 0){
        texUnit = pass->getTextureUnitState(0);
	} else {
        texUnit = pass->createTextureUnitState();
	}
    texUnit->setTextureName(texture->getName());
    texUnit->setTextureFiltering( Ogre::TFO_NONE );

	// Create the manual object
	std::stringstream ss3;
	ss3 << "MapObject-" << imageNode->getName();
	try{
        manualObject = sceneManager->createManualObject(ss3.str());
	} catch(Ogre::Exception& e) {
		sceneManager->destroyManualObject(ss3.str());
        manualObject = sceneManager->createManualObject(ss3.str());
	}
    imageNode->attachObject(manualObject);
	
	// Define the manual object as a rectangle
    manualObject->begin(material->getName(),
		Ogre::RenderOperation::OT_TRIANGLE_LIST);
	{
        // First triangle
        {
            // Bottom left
            manualObject->position( -mobots_common::constants::image_width_in_meters/2,
                                      -mobots_common::constants::image_height_in_meters/2,
                                      0.0f );
            manualObject->textureCoord(0.0f, 1.0f);
            manualObject->normal( 0.0f, 0.0f, 1.0f );
            // Top right
            manualObject->position( mobots_common::constants::image_width_in_meters/2,
                                      mobots_common::constants::image_height_in_meters/2,
                                      0.0f );
            manualObject->textureCoord(1.0f, 0.0f);
            manualObject->normal( 0.0f, 0.0f, 1.0f );
            // Top left
            manualObject->position( -mobots_common::constants::image_width_in_meters/2,
                                      mobots_common::constants::image_height_in_meters/2,
                                      0.0f );
            manualObject->textureCoord(0.0f, 0.0f);
            manualObject->normal( 0.0f, 0.0f, 1.0f );
        }
        // Second triangle
        {
            // Bottom left
            manualObject->position( -mobots_common::constants::image_width_in_meters/2,
                                      -mobots_common::constants::image_height_in_meters/2,
                                      0.0f );
            manualObject->textureCoord(0.0f, 1.0f);
            manualObject->normal( 0.0f, 0.0f, 1.0f );
            // Bottom right
            manualObject->position( mobots_common::constants::image_width_in_meters/2,
                                      -mobots_common::constants::image_height_in_meters/2,
                                      0.0f );
            manualObject->textureCoord(1.0f, 1.0f);
            manualObject->normal( 0.0f, 0.0f, 1.0f );
            // Top right
            manualObject->position( mobots_common::constants::image_width_in_meters/2,
                                      mobots_common::constants::image_height_in_meters/2,
                                      0.0f );
            manualObject->textureCoord(1.0f, 0.0f);
            manualObject->normal( 0.0f, 0.0f, 1.0f );
        }
	}
    manualObject->end();
    setImagePose(sessionID, mobotID, imageID, poseX, poseY, poseTheta, RELATIVE_POSE_NODE);
    display->sendInfoUpdate(sessionID, mobotID, ENABLED, 1);
    display->sendInfoUpdate(sessionID, mobotID, RELATIVE, 1);
    Ogre::Node* imageNodeParent= imageNode->getParent();
    display->sendInfoUpdate(sessionID, mobotID, IMAGES, imageNodeParent->numChildren());
	return 0;
}

// Make an image visible
int ImageMapVisual::showImage(int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
	imageNode->setVisible(true, true);
	return 0;
}
// Make an image invisible
int ImageMapVisual::hideImage(int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
	imageNode->setVisible(false, true);
	return 0;
}
// Delete an image and its resources.
int ImageMapVisual::deleteImage(const std::string* nodeName){
	sceneManager->destroyManualObject("MapObject-" + *nodeName);
	Ogre::TextureManager::getSingleton().remove("MapTexture-" + *nodeName);
	Ogre::MaterialManager::getSingleton().remove("MapMaterial-" + *nodeName);
	sceneManager->destroySceneNode(*nodeName);
	return 0;
}
int ImageMapVisual::deleteImage(int sessionID, int mobotID, int imageID){
    Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
    deleteImage(&imageNode->getName());
    return 0;
}

// Make all images of a mobot visible
int ImageMapVisual::showMobotImages(int sessionID, int mobotID){
    Ogre::SceneNode* mobotNode = findNode(sessionID, mobotID, -1);
    if(mobotNode == NULL){
        return 1;
    }
    mobotNode->setVisible(true, true);
    display->sendInfoUpdate(sessionID, mobotID, ENABLED, 1);
	return 0;
}
// Make all images of a mobot invisible
int ImageMapVisual::hideMobotImages(int sessionID, int mobotID){
    Ogre::SceneNode* mobotNode = findNode(sessionID, mobotID, -1);
    if(mobotNode == NULL){
        return 1;
    }
    mobotNode->setVisible(false, true);
    display->sendInfoUpdate(sessionID, mobotID, ENABLED, 0);
	return 0;
}
// Delete all images belonging to a mobot
int ImageMapVisual::deleteMobotImages(const std::string* nodeName){
	Ogre::SceneNode* mobotNode = sceneManager->getSceneNode(*nodeName);
	Ogre::Node::ChildNodeIterator imageIterator = mobotNode->getChildIterator();
	Ogre::SceneNode* imageNode;
	while(imageIterator.hasMoreElements()){
		imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
		deleteImage(&imageNode->getName());
	}
	sceneManager->destroySceneNode(*nodeName);
	return 0;
}
int ImageMapVisual::deleteMobotImages(int sessionID, int mobotID){
    Ogre::SceneNode* mobotNode = findNode(sessionID, mobotID, -1);
    if(mobotNode == NULL){
        return 1;
    }
    int result = deleteMobotImages(&mobotNode->getName());
    display->sendInfoUpdate(sessionID, mobotID*-1, 0, 0);
    return result;
}

// Make all images of a session visible
int ImageMapVisual::showSessionImages(int sessionID){
    Ogre::SceneNode* sessionNode = findNode(sessionID, -1, -1);
    if(sessionNode == NULL){
        return 1;
    }
    sessionNode->setVisible(true, true);
	return 0;
}
// Make all images of a session invisible
int ImageMapVisual::hideSessionImages(int sessionID){
    Ogre::SceneNode* sessionNode = findNode(sessionID, -1, -1);
    if(sessionNode == NULL){
        return 1;
    }
    sessionNode->setVisible(false, true);
	return 0;
}
// Delete all images belonging to a session
int ImageMapVisual::deleteSessionImages(const std::string* nodeName){
	Ogre::SceneNode* sessionNode = sceneManager->getSceneNode(*nodeName);
	Ogre::Node::ChildNodeIterator mobotIterator = sessionNode->getChildIterator();
	Ogre::SceneNode* mobotNode;
	while(mobotIterator.hasMoreElements()){
		mobotNode = static_cast<Ogre::SceneNode*> (mobotIterator.getNext());
        deleteMobotImages(&mobotNode->getName());
	}
	sceneManager->destroySceneNode(*nodeName);
	return 0;
}
// Delete all images belonging to a session
int ImageMapVisual::deleteSessionImages(int sessionID){
    Ogre::SceneNode* sessionNode = findNode(sessionID, -1, -1);
    if(sessionNode == NULL){
        return 1;
    }

    int result = deleteSessionImages(&sessionNode->getName());
    display->sendInfoUpdate(sessionID*-1, 0, 0, 0);
    return result;
}
/**
 * Deletes all nodes, images, and resources except the rootNode.
 */
int ImageMapVisual::deleteAllImages(){
    Ogre::Node::ChildNodeIterator sessionIterator = rootImageNode->getChildIterator();
	Ogre::SceneNode* sessionNode;
	while(sessionIterator.hasMoreElements()){
		sessionNode = static_cast<Ogre::SceneNode*> (sessionIterator.getNext());
        deleteSessionImages(&sessionNode->getName());
	}
	return 0;
}

/**********************************************************
  * Pose Interface
  ********************************************************/

// Position and orientation are passed through to the SceneNode
int ImageMapVisual::setImagePose(int sessionID, int mobotID, int imageID,
                                 float poseX, float poseY, float poseTheta, int poseType){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
    std::string poseName = imageNode->getName();
    if(poseType != -1){
        if(poseType == RELATIVE_POSE_NODE){
            poseName += "r";
        } else if(poseType == ABSOLUTE_POSE_NODE){
            poseName += "a";
        } else {
            ROS_ERROR("[Rviz] Invalid pose Type");
            return 1;
        }
        poseT pose = {poseX, poseY, poseTheta};
        poseMap[poseName] = pose;
    }

	// Set the orientation (theta)
	Ogre::Radian rad(poseTheta);
    Ogre::Quaternion quat(rad, Ogre::Vector3::UNIT_Z);
	imageNode->setOrientation(quat);
	// Set the position (x and y)
	Ogre::Vector3 vect(poseX, poseY, 0);
	imageNode->setPosition(vect);
    switch(poseType){
    case ABSOLUTE_POSE_NODE:
        display->sendInfoUpdate(sessionID, mobotID, ABSOLUTE, 1);
        break;
    case RELATIVE_POSE_NODE:
        display->sendInfoUpdate(sessionID, mobotID, RELATIVE, 0);
        break;
    default:
        ROS_WARN("[Rviz] Unknown pose type: setImagePose");
        break;
    }
    return 0;
}

int ImageMapVisual::imageToAbsPose(int sessionID, int mobotID, int imageID){
    Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
    return imageToPose(imageNode, RELATIVE_POSE_NODE);
}
int ImageMapVisual::imageToRelPose(int sessionID, int mobotID, int imageID){
    Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
    return imageToPose(imageNode, ABSOLUTE_POSE_NODE);
}
int ImageMapVisual::imageToPose(Ogre::SceneNode* imageNode, int poseType){
    if(imageNode == NULL){
        ROS_WARN("[Rviz] Unsafe call to private method: imageToPose");
        return 1;
    }
    std::string nodeName = imageNode->getName();
    if(poseType == RELATIVE_POSE_NODE){
        nodeName += "r";
    } else if(poseType == ABSOLUTE_POSE_NODE){
        nodeName += "a";
    } else {
        ROS_ERROR("[Rviz] Invalid pose type: imageToPose(%s, %i)",
                nodeName.c_str(), poseType);
        return 1;
    }
    if(poseMap.find(nodeName) == poseMap.end()){
        ROS_INFO("[Rviz] Pose not found in map: imageToPose(%s, %i)",
                 nodeName.c_str(), poseType);
        return 1;
    }
    poseT pose = poseMap[nodeName];
    // Set the orientation (theta)
    Ogre::Radian rad(pose.theta);
    Ogre::Quaternion quat(rad, Ogre::Vector3::UNIT_Z);
    imageNode->setOrientation(quat);
    // Set the position (x and y)
    Ogre::Vector3 vect(pose.x, pose.y, 0);
    imageNode->setPosition(vect);
    return 0;
}

int ImageMapVisual::mobotToAbsPose(int sessionID, int mobotID){
    Ogre::SceneNode* mobotNode = findNode(sessionID, mobotID, -1);
    if(mobotNode == NULL){
        return 1;
    }
    return mobotToPose(mobotNode, ABSOLUTE_POSE_NODE);
}
int ImageMapVisual::mobotToRelPose(int sessionID, int mobotID){
    Ogre::SceneNode* mobotNode = findNode(sessionID, mobotID, -1);
    if(mobotNode == NULL){
        return 1;
    }
    return mobotToPose(mobotNode, RELATIVE_POSE_NODE);
}
int ImageMapVisual::mobotToPose(Ogre::SceneNode* mobotNode, int poseType){
    Ogre::Node::ChildNodeIterator imageIterator = mobotNode->getChildIterator();
    Ogre::SceneNode* imageNode;
    while(imageIterator.hasMoreElements()){
        imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
        imageToPose(imageNode, poseType);
    }
    return 0;
}

int ImageMapVisual::sessionToAbsPose(int sessionID){
    Ogre::SceneNode* sessionNode = findNode(sessionID, -1, -1);
    if(sessionNode == NULL){
        return 1;
    }
    return sessionToPose(sessionNode, ABSOLUTE_POSE_NODE);
}
int ImageMapVisual::sessionToRelPose(int sessionID){
    Ogre::SceneNode* sessionNode = findNode(sessionID, -1, -1);
    if(sessionNode == NULL){
        return 1;
    }
    return sessionToPose(sessionNode, RELATIVE_POSE_NODE);
}
int ImageMapVisual::sessionToPose(Ogre::SceneNode* sessionNode, int poseType){
    Ogre::Node::ChildNodeIterator mobotIterator = sessionNode->getChildIterator();
    Ogre::SceneNode* mobotNode;
    while(mobotIterator.hasMoreElements()){
        mobotNode = static_cast<Ogre::SceneNode*> (mobotIterator.getNext());
        mobotToPose(mobotNode, poseType);
    }
    return 0;
}

/**********************************************************
  * Mobot Model Interface
  ********************************************************/

int ImageMapVisual::setMobotModel(int mobotID, float poseX, float poseY, float poseTheta){
    // Create SceneNode to which the model is attached
    std::string id = "model-m" + boost::lexical_cast<std::string>(mobotID);
    Ogre::Node* node;
    try{
        node = rootMobotModelNode->getChild(id);
    }catch(Ogre::Exception& e){
        rootMobotModelNode->createChildSceneNode(id, Ogre::Vector3::ZERO,
                Ogre::Quaternion::IDENTITY);
        node = rootMobotModelNode->getChild(id);
        Ogre::SceneNode* sceneNode = (Ogre::SceneNode*) node;
        createColourCube(mobotID);
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
              "Test/ColourTest-" + id, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);
        Ogre::Entity* thisEntity = sceneManager->createEntity("cc-" + id, "ColourCube-" + id);
        thisEntity->setMaterialName("Test/ColourTest-" + id);
        sceneNode->attachObject(thisEntity);
        sceneNode->setScale(0.0006, 0.0006, 0.0003);
        node = (Ogre::Node*) sceneNode;
    }
    // Set position
    Ogre::Radian rad(poseTheta);
    Ogre::Quaternion quat(rad, Ogre::Vector3::UNIT_Z);
    node->setOrientation(quat);
    // Set the position (x and y)
    Ogre::Vector3 vect(poseX, poseY, 0.5);
    node->setPosition(vect);

    return 0;
}

void ImageMapVisual::deleteMobotModel(int mobotID){
    std::string mobotName = "model-m" + boost::lexical_cast<std::string>(mobotID);
    Ogre::SceneNode* mobotNode;
    try{
        mobotNode = (Ogre::SceneNode*) rootMobotModelNode->getChild(mobotName);
    }
    catch(Ogre::Exception e){
        ROS_INFO("[Rviz] mobot model already deleted");
        return;
    }
    deleteMobotModel(&mobotNode->getName());
}
// Material: "Test/ColourTest-" + id
// Entity: "cc-" + id
// Mesh: "ColourCube" + id
void ImageMapVisual::deleteMobotModel(const std::string* nodeName){
    Ogre::MaterialManager::getSingleton().remove("Test/ColourTest-" + *nodeName);
    sceneManager->destroyEntity("cc-" + *nodeName);
    Ogre::MeshManager::getSingleton().remove("ColourCube-" + *nodeName);
    sceneManager->destroySceneNode(*nodeName);
}
/**
 * Deletes all mobot model nodes and thier resources.
 *  Mesh: "ColourCube" + id
 *  Node:
 */
void ImageMapVisual::deleteAllMobotModels(){
    Ogre::Node::ChildNodeIterator mobotModelIterator = rootMobotModelNode->getChildIterator();
    Ogre::SceneNode* mobotModelNode;
    while(mobotModelIterator.hasMoreElements()){
        mobotModelNode = static_cast<Ogre::SceneNode*> (mobotModelIterator.getNext());
        deleteMobotModel(&mobotModelNode->getName());
    }
}

void ImageMapVisual::createColourCube(int mobotID)
{
    std::string id = "-m";
    id += boost::lexical_cast<std::string>(mobotID);

    /// Create the mesh via the MeshManager
    Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourCube-model" + id, "General");

    /// Create one submesh
    Ogre::SubMesh* sub = msh->createSubMesh();

    const float sqrt13 = 0.577350269f; /* sqrt(1/3) */

    /// Define the vertices (8 vertices, each consisting of 2 groups of 3 floats
    const size_t nVertices = 8;
    const size_t vbufCount = 3*2*nVertices;
    float vertices[vbufCount] = {
            -100.0,100.0,-100.0,        //0 position
            -sqrt13,sqrt13,-sqrt13,     //0 normal
            100.0,100.0,-100.0,         //1 position
            sqrt13,sqrt13,-sqrt13,      //1 normal
            100.0,-100.0,-100.0,        //2 position
            sqrt13,-sqrt13,-sqrt13,     //2 normal
            -100.0,-100.0,-100.0,       //3 position
            -sqrt13,-sqrt13,-sqrt13,    //3 normal
            -100.0,100.0,100.0,         //4 position
            -sqrt13,sqrt13,sqrt13,      //4 normal
            100.0,100.0,100.0,          //5 position
            sqrt13,sqrt13,sqrt13,       //5 normal
            100.0,-100.0,100.0,         //6 position
            sqrt13,-sqrt13,sqrt13,      //6 normal
            -100.0,-100.0,100.0,        //7 position
            -sqrt13,-sqrt13,sqrt13,     //7 normal
    };

    Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();
    Ogre::RGBA colours[nVertices];
    Ogre::RGBA *pColour = colours;
    // Use render system to convert colour value since colour packing varies
    float color = 0.2 * mobotID + 0.1;
    while(color >= 1){
        color = color / 10;
    }
    rs->convertColourValue(Ogre::ColourValue(color, color, color), pColour++); //0 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,1.0,0.0), pColour++); //1 colour
    rs->convertColourValue(Ogre::ColourValue(color, color, color), pColour++); //2 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,0.0,0.0), pColour++); //3 colour
    rs->convertColourValue(Ogre::ColourValue(color, color, color), pColour++); //4 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,1.0,1.0), pColour++); //5 colour
    rs->convertColourValue(Ogre::ColourValue(color, color, color), pColour++); //6 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,0.0,1.0), pColour++); //7 colour

    /// Define 12 triangles (two triangles per cube face)
    /// The values in this table refer to vertices in the above table
    const size_t ibufCount = 36;
    unsigned short faces[ibufCount] = {
            0,2,3,
            0,1,2,
            1,6,2,
            1,5,6,
            4,6,5,
            4,7,6,
            0,7,4,
            0,3,7,
            0,5,1,
            0,4,5,
            2,7,3,
            2,6,7
    };

    /// Create vertex data structure for 8 vertices shared between submeshes
    msh->sharedVertexData = new Ogre::VertexData();
    msh->sharedVertexData->vertexCount = nVertices;

    /// Create declaration (memory format) of vertex data
    Ogre::VertexDeclaration* decl = msh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;
    // 1st buffer
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    Ogre::HardwareVertexBufferSharedPtr vbuf =
        Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);

    /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
    Ogre::VertexBufferBinding* bind = msh->sharedVertexData->vertexBufferBinding;
    bind->setBinding(0, vbuf);

    // 2nd buffer
    offset = 0;
    decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount)
    /// and bytes per vertex (offset)
    vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), colours, true);

    /// Set vertex buffer binding so buffer 1 is bound to our colour buffer
    bind->setBinding(1, vbuf);

    /// Allocate index buffer of the requested number of vertices (ibufCount)
    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
        createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        ibufCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    /// Upload the index data to the card
    ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

    /// Set parameters of the submesh
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = ibufCount;
    sub->indexData->indexStart = 0;

    /// Set bounding information (for culling)
    msh->_setBounds(Ogre::AxisAlignedBox(-100,-100,-100,100,100,100));
    msh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3*100*100));

    /// Notify -Mesh object that it has been loaded
    msh->load();
}


/**********************************************************
  * Node Helper Interface (private)
  ********************************************************/

/**
 * Searches for the requested Node. Creates the path if it does not exist. 
 */
Ogre::SceneNode* ImageMapVisual::getNode(int sessionID, int mobotID, int imageID){
	// Get the specified session node
	std::string name = "s";
	name += boost::lexical_cast<std::string>(sessionID);
	Ogre::Node* node;
	try{
        node = rootImageNode->getChild(name);
	}catch(Ogre::Exception& e){
        rootImageNode->createChildSceneNode(name, Ogre::Vector3::ZERO,
			Ogre::Quaternion::IDENTITY);
        node = rootImageNode->getChild(name);
	}
	// Get the specified mobot node
	name += "m";
    name += boost::lexical_cast<std::string>(mobotID);
	try{
		node = node->getChild(name);
	}catch(Ogre::Exception& e){
		node->createChild(name, Ogre::Vector3::ZERO,
			Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
	}
	// Get the specified image node
	name += "i";
	name += boost::lexical_cast<std::string>(imageID);
	try{
		node = node->getChild(name);
	}catch(Ogre::Exception& e){
		node->createChild(name, Ogre::Vector3::ZERO,
			Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
	}
	return (Ogre::SceneNode*) node;
}

/**
 * Searches for the requested Node. Returns NULL pointer if node is not found. 
 */
Ogre::SceneNode* ImageMapVisual::findNode(int sessionID, int mobotID, int imageID){
    ROS_INFO("[Find Node] (%i,%i,%i)", sessionID, mobotID, imageID);
	// Get the specified session node
	if(sessionID < 0){
        ROS_ERROR("[Rviz] find node: Not found");
		return NULL;
	}
	std::string name = "s";
	name += boost::lexical_cast<std::string>(sessionID);
	Ogre::Node* node;
	try{
        node = rootImageNode->getChild(name);
	} catch(Ogre::Exception& e) {
        ROS_ERROR("[Rviz] find node: Error %s", name.c_str());
		return NULL;
	}
	// Get the specified mobot node
	if(mobotID < 0){
		return (Ogre::SceneNode*) node;
	}
	name += "m";
    name += boost::lexical_cast<std::string>(mobotID);
	try{
		node = node->getChild(name);
	} catch(Ogre::Exception& e) {
        ROS_ERROR("[Rviz] find node: Error %s", name.c_str());
		return NULL;
	}
	// Get the specified image node
	if(imageID < 0){
		return (Ogre::SceneNode*) node;
	}
	name += "i";
	name += boost::lexical_cast<std::string>(imageID);
	try{
		node = node->getChild(name);
	} catch(Ogre::Exception& e) {
        ROS_ERROR("[Rviz] find node: Error %s", name.c_str());
		return NULL;
	}
	return (Ogre::SceneNode*) node;
}

}
