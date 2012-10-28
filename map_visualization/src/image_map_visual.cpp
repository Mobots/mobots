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
    rootImageNode = sceneManager->getRootSceneNode()->createChildSceneNode();
    rootMobotModelNode = sceneManager->getRootSceneNode()->createChildSceneNode();
}

/**
 * All resources(nodes, textures, material) are deleted.
 */
ImageMapVisual::~ImageMapVisual()
{
    ROS_INFO("~visual");
    deleteAllImages();
    ROS_INFO("Check1");
    deleteAllMobotModels();
    ROS_INFO("Check2");
    sceneManager->destroySceneNode(rootImageNode);
    ROS_INFO("Check3");
    sceneManager->destroySceneNode(rootMobotModelNode);
    ROS_INFO("~visual");
}

/**
 * Insert an image into Rviz and set the pose.
 * @Notes All resources are named;enabling a clean deletion
 * TODO variable image resolution
 */
int ImageMapVisual::insertImage(float poseX, float poseY, float poseTheta,
    int sessionID, int mobotID,	int imageID, cv::Mat mat)
{
	// Get the node to which the image is assigned to
	Ogre::SceneNode* imageNode = getNode(sessionID, mobotID, imageID);
	// Deleting configuration and resources of a node with the same ID
	Ogre::SceneNode* parentNode = imageNode->getParentSceneNode();
	std::string imageNodeName = imageNode->getName();
	deleteImage(&imageNodeName);
    imageNode = parentNode->createChildSceneNode(imageNodeName,
		Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
    //	ROS_INFO("insertImage, imageNode: %s", (imageNode->getName()).c_str());

    cv::namedWindow("recieved_image", 1);
    cv::imshow("recieved_image", mat);
	
	// Create the material
	std::stringstream ss;
	ss << "MapMaterial-" << imageNode->getName();
    ROS_INFO("%s", (ss.str()).c_str());
	try{
		material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	} catch(Ogre::Exception& e) {
//		ROS_INFO("[Visual] %s", e.what());
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
//	ROS_INFO("%s", (ss2.str()).c_str());
	try{
		texture_ = Ogre::TextureManager::getSingleton().createManual(
			ss2.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
			Ogre::TEX_TYPE_2D, mat.cols, mat.rows, 0, Ogre::PF_X8R8G8B8,
			Ogre::TU_DEFAULT);
    } catch(Ogre::Exception& e) {
//		ROS_INFO("[Visual] %s", e.what());
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
//	ROS_INFO("object: %s", ss3.str().c_str());
	try{
		manual_object_ = sceneManager->createManualObject(ss3.str());
	} catch(Ogre::Exception& e) {
//		ROS_INFO("[Visual] object: %s", e.what());
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
    setImagePose(poseX, poseY, poseTheta, sessionID, mobotID, imageID);
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
//	ROS_INFO("[deleteImage]");
	sceneManager->destroyManualObject("MapObject-" + *nodeName);
	Ogre::TextureManager::getSingleton().remove("MapTexture-" + *nodeName);
	Ogre::MaterialManager::getSingleton().remove("MapMaterial-" + *nodeName);
	sceneManager->destroySceneNode(*nodeName);
//	ROS_INFO("[deleteImage]");
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
//	ROS_INFO("[deleteMobot]");
	Ogre::SceneNode* mobotNode = sceneManager->getSceneNode(*nodeName);
	Ogre::Node::ChildNodeIterator imageIterator = mobotNode->getChildIterator();
	Ogre::SceneNode* imageNode;
	while(imageIterator.hasMoreElements()){
		imageNode = static_cast<Ogre::SceneNode*> (imageIterator.getNext());
		deleteImage(&imageNode->getName());
	}
	sceneManager->destroySceneNode(*nodeName);
//	ROS_INFO("[deleteMobot]");
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
//	ROS_INFO("[deleteSession]");
	Ogre::SceneNode* sessionNode = sceneManager->getSceneNode(*nodeName);
	Ogre::Node::ChildNodeIterator mobotIterator = sessionNode->getChildIterator();
	Ogre::SceneNode* mobotNode;
	while(mobotIterator.hasMoreElements()){
		mobotNode = static_cast<Ogre::SceneNode*> (mobotIterator.getNext());
		deleteMobot(&mobotNode->getName());
	}
	sceneManager->destroySceneNode(*nodeName);
//	ROS_INFO("[deleteSession]");
	return 0;
}
/**
 * Deletes all nodes, images, and resources except the rootNode.
 */
int ImageMapVisual::deleteAllImages(){
//	ROS_INFO("[deleteAllImages]");
    Ogre::Node::ChildNodeIterator sessionIterator = rootImageNode->getChildIterator();
	Ogre::SceneNode* sessionNode;
	while(sessionIterator.hasMoreElements()){
		sessionNode = static_cast<Ogre::SceneNode*> (sessionIterator.getNext());
		deleteSession(&sessionNode->getName());
	}
//	ROS_INFO("[deleteAllImages]");
	return 0;
}

// Position and orientation are passed through to the SceneNode
int ImageMapVisual::setImagePose(float poseX, float poseY, float poseTheta,
									int sessionID, int mobotID, int imageID){
	Ogre::SceneNode* imageNode = findNode(sessionID, mobotID, imageID);
    if(imageNode == NULL){
        return 1;
    }
	// Set the orientation (theta)
	Ogre::Radian rad(poseTheta);
    Ogre::Quaternion quat(rad, Ogre::Vector3::UNIT_Z);
	imageNode->setOrientation(quat);
	// Set the position (x and y)
	Ogre::Vector3 vect(poseX, poseY, 0);
	imageNode->setPosition(vect);
    return 0;
}

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
        sceneNode->setScale(0.01, 0.01, 0.005);
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
 * Searches for the requested Node. Creates the path if it does not exist. 
 */
Ogre::SceneNode* ImageMapVisual::getNode(int sessionID, int mobotID, int imageID){
    ROS_INFO("[Get Node]");
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
    ROS_INFO("[Find Node]");
	// Get the specified session node
	if(sessionID < 0){
		return NULL;
	}
	std::string name = "s";
	name += boost::lexical_cast<std::string>(sessionID);
	Ogre::Node* node;
	try{
        node = rootImageNode->getChild(name);
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
    rs->convertColourValue(Ogre::ColourValue(1.0,0.0,0.0), pColour++); //0 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,1.0,0.0), pColour++); //1 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,1.0,0.0), pColour++); //2 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,0.0,0.0), pColour++); //3 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,0.0,1.0), pColour++); //4 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,1.0,1.0), pColour++); //5 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,1.0,1.0), pColour++); //6 colour
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

}
