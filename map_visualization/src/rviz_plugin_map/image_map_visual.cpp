#include "image_map_visual.h"

namespace map_visualization
{

/**
 * Creates the root node. Images are placed into a tree hierarchy.
 * root -> sessions -> mobots -> images
 */
ImageMapVisual::ImageMapVisual( Ogre::SceneManager* sceneManager_){
	sceneManager = sceneManager_;
	// The root of the node tree containing all image nodes 
	rootNode = sceneManager->getRootSceneNode()->createChildSceneNode();
}

/**
 * All resources(nodes, textures, material) are deleted.
 */
ImageMapVisual::~ImageMapVisual()
{
	ROS_INFO("~visual");
	deleteAllImages();
	sceneManager->destroySceneNode(rootNode);
	ROS_INFO("~visual");
}

/**
 * Insert an image into Rviz and set the pose.
 * @Notes All resources are named;enabling a clean deletion
 * TODO variable image resolution
 */
int ImageMapVisual::insertImage(
	float poseX, float poseY, float poseTheta,
	int sessionID, int mobotID, int imageID,
	const std::vector<unsigned char>* imageData,
	const std::string* encoding, int width, int height
){
	// Get the node to which the image is assigned to
	Ogre::SceneNode* imageNode = getNode(sessionID, mobotID, imageID);
	// Deleting configuration and resources of a node with the same ID
	Ogre::SceneNode* parentNode = imageNode->getParentSceneNode();
	std::string imageNodeName = imageNode->getName();
	deleteImage(&imageNodeName);
	imageNode = parentNode->createChildSceneNode(imageNodeName,
		Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
	ROS_INFO("insertImage, imageNode: %s", (imageNode->getName()).c_str());
	
	// Decode the [png/jpg] image to RGB encoding
	cv::Mat mat = cv::imdecode(*imageData, 1);
	cv::namedWindow("window_title", 1);
	cv::imshow("window_title", mat);
	
	// Create the material
	std::stringstream ss;
	ss << "MapMaterial-" << imageNode->getName();
	ROS_INFO("%s", (ss.str()).c_str());
	try{
		material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	} catch(Ogre::Exception& e) {
		ROS_INFO("[Visual] %s", e.what());
		Ogre::MaterialManager::getSingleton().remove(ss.str());
		material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	}
	material_->setReceiveShadows(false);
	material_->getTechnique(0)->setLightingEnabled(false);
	material_->setDepthBias(-16.0f, 0.0f);
	material_->setCullingMode(Ogre::CULL_NONE);
	material_->setDepthWriteEnabled(false);
	
	// Create the texture
	std::stringstream ss2;
	ss2 << "MapTexture-" << imageNode->getName();
	ROS_INFO("%s", (ss2.str()).c_str());
	try{
		texture_ = Ogre::TextureManager::getSingleton().createManual(
			ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, mat.cols, mat.rows, 0, Ogre::PF_X8R8G8B8,
			Ogre::TU_DEFAULT);
	} catch(Ogre::Exception& e) {
		ROS_INFO("[Visual] %s", e.what());
		Ogre::TextureManager::getSingleton().remove(ss2.str());
		texture_ = Ogre::TextureManager::getSingleton().createManual(
			ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, mat.cols, mat.rows, 0, Ogre::PF_X8R8G8B8,
			Ogre::TU_DEFAULT);
	}
	
	// Get pixel buffer
	Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();
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
	Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
	if (pass->getNumTextureUnitStates() > 0){
		tex_unit_ = pass->getTextureUnitState(0);
	} else {
		tex_unit_ = pass->createTextureUnitState();
	}
	tex_unit_->setTextureName(texture_->getName());
	tex_unit_->setTextureFiltering( Ogre::TFO_NONE );

	// Create the manual object
	std::stringstream ss3;
	ss3 << "MapObject-" << imageNode->getName();
	ROS_INFO("object: %s", ss3.str().c_str());
	try{
		manual_object_ = sceneManager->createManualObject(ss3.str());
	} catch(Ogre::Exception& e) {
		ROS_INFO("[Visual] object: %s", e.what());
		sceneManager->destroyManualObject(ss3.str());
		manual_object_ = sceneManager->createManualObject(ss3.str());
	}
	imageNode->attachObject(manual_object_);
	
	// Normalize the image size
	float imageScale = 5;
	float widthScaled = imageScale;
  float rows = mat.rows;
  float cols = mat.cols;
	float heightScaled = (rows / cols) * imageScale;
	std::cout << imageScale << " " << mat.cols << " " << widthScaled << " " << mat.rows << " " << heightScaled << std::endl;
	
	// Define the manual object as a rectangle
	manual_object_->begin(material_->getName(),
		Ogre::RenderOperation::OT_TRIANGLE_LIST);
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
// Delete an image and its resources.
int ImageMapVisual::deleteImage(const std::string* nodeName){
	ROS_INFO("[deleteImage]");
	sceneManager->destroyManualObject("MapObject-" + *nodeName);
	Ogre::TextureManager::getSingleton().remove("MapTexture-" + *nodeName);
	Ogre::MaterialManager::getSingleton().remove("MapMaterial-" + *nodeName);
	sceneManager->destroySceneNode(*nodeName);
	ROS_INFO("[deleteImage]");
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
int ImageMapVisual::deleteMobot(const std::string* nodeName){
	ROS_INFO("[deleteMobot]");
	Ogre::SceneNode* mobotNode = sceneManager->getSceneNode(*nodeName);
	Ogre::Node::ChildNodeIterator imageIterator = mobotNode->getChildIterator();
	Ogre::SceneNode* imageNode;
	while(imageIterator.hasMoreElements()){
		ROS_INFO("Iterator");
		imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
		deleteImage(&imageNode->getName());
	}
	sceneManager->destroySceneNode(*nodeName);
	ROS_INFO("[deleteMobot]");
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
// Delete all images belonging to a session
int ImageMapVisual::deleteSession(const std::string* nodeName){
	ROS_INFO("[deleteSession]");
	Ogre::SceneNode* sessionNode = sceneManager->getSceneNode(*nodeName);
	Ogre::Node::ChildNodeIterator mobotIterator = sessionNode->getChildIterator();
	Ogre::SceneNode* mobotNode;
	while(mobotIterator.hasMoreElements()){
		ROS_INFO("Iterator");
		mobotNode = static_cast<Ogre::SceneNode*> (mobotIterator.getNext());
		deleteMobot(&mobotNode->getName());
	}
	sceneManager->destroySceneNode(*nodeName);
	ROS_INFO("[deleteSession]");
	return 0;
}
/**
 * Deletes all nodes, images, and resources except the rootNode.
 */
int ImageMapVisual::deleteAllImages(){
	ROS_INFO("[deleteAllImages]");
	Ogre::Node::ChildNodeIterator sessionIterator = rootNode->getChildIterator();
	Ogre::SceneNode* sessionNode;
	while(sessionIterator.hasMoreElements()){
		ROS_INFO("Iterator");
		sessionNode = static_cast<Ogre::SceneNode*> (sessionIterator.getNext());
		deleteSession(&sessionNode->getName());
	}
	ROS_INFO("[deleteAllImages]");
	return 0;
}

// Position and orientation are passed through to the SceneNode.
void ImageMapVisual::setPose(float poseX, float poseY, float poseTheta,
									int sessionID, int mobotID, int imageID){
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
 * Searches for the requested Node. Creates the path if it does not exist. 
 */
Ogre::SceneNode* ImageMapVisual::getNode(int sessionID, int mobotID, int imageID){
	// Get the specified session node
	std::string name = "s";
	name += boost::lexical_cast<std::string>(sessionID);
	Ogre::Node* node;
	try{
		node = rootNode->getChild(name);
	}catch(Ogre::Exception& e){
		rootNode->createChildSceneNode(name, Ogre::Vector3::ZERO,
			Ogre::Quaternion::IDENTITY);
		node = rootNode->getChild(name);
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
	// Get the specified session node
	if(sessionID < 0){
		return NULL;
	}
	std::string name = "s";
	name += boost::lexical_cast<std::string>(sessionID);
	Ogre::Node* node;
	try{
		node = rootNode->getChild(name);
	} catch(Ogre::Exception& e) {
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
		return NULL;
	}
	return (Ogre::SceneNode*) node;
}

}
