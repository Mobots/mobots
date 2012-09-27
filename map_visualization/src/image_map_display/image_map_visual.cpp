#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreDataStream.h>
#include <OGRE/OgreHardwarePixelBuffer.h>

#include <iostream>
#include <fstream>
#include <list>

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include "image_map_visual.h"

namespace map_visualization
{

ImageMapVisual::ImageMapVisual( Ogre::SceneManager* sceneManager_, Ogre::SceneNode* parent_node ){
	sceneManager = sceneManager_;
	rootNode = sceneManager->getRootSceneNode()->createChildSceneNode();;
}

// delete: nodes->objects->textures->materials
ImageMapVisual::~ImageMapVisual()
{
	ROS_INFO("~visual");
	rootNode->removeAndDestroyAllChildren();
	sceneManager->destroySceneNode(rootNode);
	//sceneManager->destroyAllManualObjects();
	ROS_INFO("~visual");
}

/**
 * TODO variable image resolution
 */
int ImageMapVisual::insertImage(
	float poseX, float poseY, float poseTheta,
	int sessionID, int mobotID, int imageID,
	const std::vector<unsigned char>* imageData,
	const std::string* encoding, int width, int height
){
	// Get the node to which the image shall be assigned to
	ROS_INFO("Check 1.0");
	Ogre::SceneNode* imageNode = getNode(sessionID, mobotID, imageID);
	ROS_INFO("insertImage, image node name: %s", (imageNode->getName()).c_str());
	static int count = 0;
	std::stringstream ss;
	ss << "MapMaterial" << count++;
	ROS_INFO("material: %s", (ss.str()).c_str());
	material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	material_->setReceiveShadows(false);
	material_->getTechnique(0)->setLightingEnabled(false);
	material_->setDepthBias( -16.0f, 0.0f );
	material_->setCullingMode( Ogre::CULL_NONE );
	material_->setDepthWriteEnabled(false);
	
	static int tex_count = 0;
	std::stringstream ss2;
	ss2 << "MapTexture" << tex_count++;
	
	//IplImage* img = cvLoadImage( "/home/moritz/TillEvil.jpg", 1);
	cv::Mat mat = cv::imdecode(*imageData, 1);
	//cvShowImage("mainWin", &mat );
	
	// Create the texture
	ROS_INFO("texture: %s", (ss2.str()).c_str());
	texture_ = Ogre::TextureManager::getSingleton().createManual(
		ss2.str(),				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D, 		// type
		mat.cols, mat.rows,// width & height
		0,						// No. of mipmaps
		Ogre::PF_X8R8G8B8,		// PixelFormat
		Ogre::TU_DEFAULT);
	//---------------------------------------------------------
	// Get the pixel buffer
	Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_
	->getBuffer();
	
	// Lock the pixel buffer and get a pixel box
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
	
	// Unlock the pixel buffer
	pixelBuffer->unlock();
	
	// Create a material using the texture
	material_ = Ogre::MaterialManager::getSingleton().create(
		"DynamicTextureMaterial", // name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	//---------------------------------------------------------
	Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
	if (pass->getNumTextureUnitStates() > 0){
		tex_unit_ = pass->getTextureUnitState(0);
	} else {
		tex_unit_ = pass->createTextureUnitState();
	}
	tex_unit_->setTextureName(texture_->getName());
	tex_unit_->setTextureFiltering( Ogre::TFO_NONE );

	static int object_count = 0;	
	std::stringstream ss3;
	ss3 << "MapObject" << object_count++;
	ROS_INFO("manual object: %s", (ss3.str()).c_str());
	manual_object_ = sceneManager->createManualObject(ss3.str());
	imageNode->attachObject(manual_object_);
	
	float imageScale = 5;
	float widthScaled = imageScale;
	float heightScaled = (mat.rows / mat.cols) * imageScale;
	std::cout << imageScale << " " << mat.cols << " " << widthScaled << " " << mat.rows << " " << heightScaled;
	
	manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
	{
		// First triangle
		{
			// Bottom left
			manual_object_->position( 0.0f, 0.0f, 0.0f );
			manual_object_->textureCoord(0.0f, 1.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Top right
			manual_object_->position( widthScaled, heightScaled, 0.0f );
			manual_object_->textureCoord(1.0f, 0.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Top left
			manual_object_->position( 0.0f, heightScaled, 0.0f );
			manual_object_->textureCoord(0.0f, 0.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
		}
		
		// Second triangle
		{
			// Bottom left
			manual_object_->position( 0.0f, 0.0f, 0.0f );
			manual_object_->textureCoord(0.0f, 1.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Bottom right
			manual_object_->position( widthScaled, 0.0f, 0.0f );
			manual_object_->textureCoord(1.0f, 1.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Top right
			manual_object_->position( widthScaled, heightScaled, 0.0f );
			manual_object_->textureCoord(1.0f, 0.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
		}
	}
	manual_object_->end();
	setPose(poseX, poseY, poseTheta, sessionID, mobotID, imageID);
	return 0;
}

// Make an image visible
int ImageMapVisual::showImage(int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
	imageNode->setVisible(true, true);
	return 0;
}
// Make an image invisible
int ImageMapVisual::hideImage(int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
	imageNode->setVisible(false, true);
	return 0;
}
// Delete an image
int ImageMapVisual::deleteImage(int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
	if(imageNode == NULL){
		return 1;
	}
	sceneManager->destroyMovableObject(imageNode->getAttachedObject(0));
	sceneManager->destroySceneNode(imageNode);
	return 0;
}

// Make all images of a mobot visible
int ImageMapVisual::showMobot(int sessionID, int mobotID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, -1);
	imageNode->setVisible(true, true);
	return 0;
}
// Make all images of a mobot invisible
int ImageMapVisual::hideMobot(int sessionID, int mobotID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, -1);
	imageNode->setVisible(false, true);
	return 0;
}
// Delete all images belonging to a mobot
int ImageMapVisual::deleteMobot(int sessionID, int mobotID){
	Ogre::SceneNode* mobotNode = findNode(sessionID, -1, -1);
	if(mobotNode == NULL){
		return 1;
	}
	Ogre::Node::ChildNodeIterator imageIterator = mobotNode->getChildIterator();
	Ogre::SceneNode* imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
	while(imageIterator.hasMoreElements()){
		sceneManager->destroyMovableObject(imageNode->getAttachedObject(0));
		imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
	}
	imageNode->removeAndDestroyAllChildren();
	sceneManager->destroySceneNode(imageNode);
	return 0;
}

// Make all images of a session visible
int ImageMapVisual::showSession(int sessionID){
	Ogre::SceneNode* imageNode = findNode(sessionID, -1, -1);
	imageNode->setVisible(true, true);
	return 0;
}
// Make all images of a session invisible
int ImageMapVisual::hideSession(int sessionID){
	Ogre::SceneNode* imageNode = findNode(sessionID, -1, -1);
	imageNode->setVisible(false, true);
	return 0;
}

int ImageMapVisual::deleteSession(int sessionID){
	Ogre::SceneNode* sessionNode = findNode(sessionID, -1, -1);
	if(sessionNode == NULL){
		return 1;
	}
	Ogre::Node::ChildNodeIterator mobotIterator = sessionNode->getChildIterator();
	Ogre::Node* mobotNode;
	// Don't know how to initialize child iterator :/
	Ogre::Node::ChildNodeIterator imageIterator = sessionNode->getChildIterator();
	Ogre::SceneNode* imageNode;
	while(mobotIterator.hasMoreElements()){
		mobotNode = mobotIterator.getNext();
		imageIterator = mobotNode->getChildIterator();
		while(imageIterator.hasMoreElements()){
			imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
			sceneManager->destroyMovableObject(imageNode->getAttachedObject(0));
		}
	}
	sessionNode->removeAndDestroyAllChildren();
	sceneManager->destroySceneNode(sessionNode);
	return 0;
}

// Position and orientation are passed through to the SceneNode.
void ImageMapVisual::setPose(float poseX, float poseY, float poseTheta, int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
	// Set the orientation (theta)
	Ogre::Radian rad(poseTheta);
	Ogre::Quaternion quat(rad, Ogre::Vector3::UNIT_Y);
	imageNode->setOrientation(quat);
	// Set the position (x and y)
	Ogre::Vector3 vect(poseX, poseY, 0);
	imageNode->setPosition(vect);
	return;
}

/**
 * Searches for the requested Node.
 * @remarks If the node is not found, a the node and its path is created.
 */
Ogre::SceneNode* ImageMapVisual::getNode(int sessionID, int mobotID, int imageID){
	ROS_INFO("Check 2.0: %i:%i:%i", sessionID, mobotID, imageID);
	std::string name = "s";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << sessionID))->str();
	Ogre::Node* node;
	try{
		ROS_INFO("Check 2.1");
		node = rootNode->getChild(name);
		ROS_INFO("Check 2.1");
	}catch(Ogre::Exception& e){
		ROS_INFO("Check 2.1-");
		rootNode->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = rootNode->getChild(name);
	}
	name += "m";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << mobotID))->str();
	try{
		ROS_INFO("Check 2.2");
		node = node->getChild(name);
		ROS_INFO("Check 2.2");
	}catch(Ogre::Exception& e){
		ROS_INFO("Check 2.2-");
		node->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
	}
	name += "i";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << imageID))->str();
	try{
		ROS_INFO("Check 2.3");
		node = node->getChild(name);
	}catch(Ogre::Exception& e){
		ROS_INFO("Check 2.3-");
		node->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
	}
	return (Ogre::SceneNode*) node;
	ROS_INFO("Check 2.X");
}

/**
 * Searches for the requested Node.
 * @remarks If the node is not found, a NULL pointer is returned.
 */
Ogre::SceneNode* ImageMapVisual::findNode(int sessionID, int mobotID, int imageID){
	if(sessionID < 0){
		return NULL;
	}
	std::string name = "s";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << sessionID))->str();
	Ogre::Node* node;
	try{
		node = rootNode->getChild(name);
	} catch(Ogre::Exception& e) {
		rootNode->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = rootNode->getChild(name);
	}
	if(mobotID < 0){
		return (Ogre::SceneNode*) node;
	}
	name += "m";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << mobotID))->str();
	try{
		node = node->getChild(name);
	} catch(Ogre::Exception& e) {
		node->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
	}
	if(imageID < 0){
		return (Ogre::SceneNode*) node;
	}
	name += "i";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << imageID))->str();
	try{
		node = node->getChild(name);
	} catch(Ogre::Exception& e) {
		node->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
	}
	return (Ogre::SceneNode*) node;
}

std::string ImageMapVisual::getMapObjectName(int sessionID, int mobotID, int imageID){
	std::stringstream ss;
	ss << "MapObject:s" << sessionID << "m" << mobotID << "i" << imageID;
	return ss.str();
}

}

