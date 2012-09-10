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
	rootNode = parent_node->createChildSceneNode();
}

// delete: nodes->objects->textures->materials
ImageMapVisual::~ImageMapVisual()
{
	ROS_INFO("~visual");
	while(!manualObjects.empty()){
		sceneManager->destroyManualObject(manualObjects.front());
		manualObjects.pop_front();
		ROS_INFO("object deleted");
	}
	ROS_INFO("remove all children");
	rootNode->removeAndDestroyAllChildren();
	ROS_INFO("remove root");
	sceneManager->destroySceneNode(rootNode);
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
	Ogre::SceneNode* imageNode = getImageNode(sessionID, mobotID, imageID);
	ROS_INFO("insertImage, image node name: %s", (imageNode->getName()).c_str());
	static int count = 0;
	std::stringstream ss;
	ss << "MapObjectMaterial" << count++;
	material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	material_->setReceiveShadows(false);
	material_->getTechnique(0)->setLightingEnabled(false);
	material_->setDepthBias( -16.0f, 0.0f );
	material_->setCullingMode( Ogre::CULL_NONE );
	material_->setDepthWriteEnabled(false);
	
	static int tex_count = 0;
	std::stringstream ss2;
	ss2 << "MapTexture" << tex_count++;
	
	IplImage* img = cvLoadImage( "/home/moritz/TillEvil.jpg", 1);
	//cvShowImage("mainWin", img );
	
	// Create the texture
	texture_ = Ogre::TextureManager::getSingleton().createManual(
		ss2.str(),				// name
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D, 		// type
		img->width, img->height,// width & height
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
	for (int j = 0; j < img->width * img->height; j++){
		bit32 = ((img->imageData[k]&0xFF)<<16) | ((img->imageData[k + 1]&0xFF)<<8) | ((img->imageData[k + 2]&0xFF));
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
	if (pass->getNumTextureUnitStates() > 0)
	{
		tex_unit_ = pass->getTextureUnitState(0);
	}
	else
	{
		tex_unit_ = pass->createTextureUnitState();
	}
	tex_unit_->setTextureName(texture_->getName());
	tex_unit_->setTextureFiltering( Ogre::TFO_NONE );
	
	static int map_count = 0;
	std::stringstream ss3;
	ss3 << "MapObject" << map_count++;
	manual_object_ = sceneManager->createManualObject( ss3.str() );
	imageNode->attachObject( manual_object_ );
	manualObjects.push_back(manual_object_);
	
	width_ = 5;
	height_ = 5;
	
	manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
	{
		// First triangle
		{
			// Bottom left
			manual_object_->position( 0.0f, 0.0f, 0.0f );
			manual_object_->textureCoord(0.0f, 1.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Top right
			manual_object_->position( width_, height_, 0.0f );
			manual_object_->textureCoord(1.0f, 0.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Top left
			manual_object_->position( 0.0f, height_, 0.0f );
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
			manual_object_->position( width_, 0.0f, 0.0f );
			manual_object_->textureCoord(1.0f, 1.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
			
			// Top right
			manual_object_->position( width_, height_, 0.0f );
			manual_object_->textureCoord(1.0f, 0.0f);
			manual_object_->normal( 0.0f, 0.0f, 1.0f );
		}
	}
	manual_object_->end();
	return 0;
}

// Position and orientation are passed through to the SceneNode.
void ImageMapVisual::setPosition(const Ogre::Vector3& position, int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = getImageNode(sessionID, mobotID, imageID);
	imageNode->setPosition(position);
}

void ImageMapVisual::setOrientation(const Ogre::Quaternion& orientation, int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = getImageNode(sessionID, mobotID, imageID);
	imageNode->setOrientation(orientation);
}

/**
 * With the ID's, the scene graph is traversed to find the requested leaf node.
 */
Ogre::SceneNode* ImageMapVisual::getImageNode(int sessionID, int mobotID, int imageID){
	std::string name = "s";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << sessionID))->str();
	Ogre::Node* node;
	try{
		node = rootNode->getChild(name);
	} catch(Ogre::Exception& e) {
		rootNode->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = rootNode->getChild(name);
	}
	name += "m";
	name += static_cast<std::ostringstream*>( &(std::ostringstream() << mobotID))->str();
	try{
		node = node->getChild(name);
	} catch(Ogre::Exception& e) {
		node->createChild(name, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
		node = node->getChild(name);
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

}

