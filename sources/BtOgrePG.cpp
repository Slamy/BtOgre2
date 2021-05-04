#include "BtOgrePG.h"

using namespace Ogre;
using namespace BtOgre;

#if BOOST_ASYNCHRONOUS == 1
boost::lockfree::queue<MotionTransfer*> RigidBodyState::motion_transfers(1024);
#endif

RigidBodyState::RigidBodyState(SceneNode* node, const btTransform& transform, const btTransform& offset) :
	mTransform(transform),
	mCenterOfMassOffset(offset),
	mNode(node)
{
}

RigidBodyState::RigidBodyState(SceneNode* node) :
	mTransform
	(
		node ? Convert::toBullet(node->_getDerivedOrientationUpdated()) : btQuaternion(0, 0, 0, 1),
		node ? Convert::toBullet(node->_getDerivedPositionUpdated()) : btVector3(0, 0, 0)
	),
	mCenterOfMassOffset(btTransform::getIdentity()),
	mNode(node)
{
}

void RigidBodyState::getWorldTransform(btTransform& ret) const
{
	ret = mTransform;
}

void RigidBodyState::setWorldTransformNoUpdate(const btTransform& in)
{
	mTransform = in;
}

void RigidBodyState::setWorldTransform(const btTransform& in)
{
	if (!mNode) return;

	//store transform
	mTransform = in;

	//extract position and orientation
	const auto transform = mTransform * mCenterOfMassOffset;
	const auto rot = transform.getRotation();
	const auto pos = transform.getOrigin();
#if BOOST_ASYNCHRONOUS == 1
	MotionTransfer* info = new MotionTransfer;
	info->mNode = mNode;
	info->pos = BtOgre::Convert::toOgre(pos);
	info->orient = BtOgre::Convert::toOgre(rot);

	motion_transfers.push(info);
#else
	//Set to the node directly
	mNode->_setDerivedOrientation({ rot.w(), rot.x(), rot.y(), rot.z() });
	mNode->_setDerivedPosition({ pos.x(), pos.y(), pos.z() });
#endif
}

void RigidBodyState::setNode(SceneNode* node)
{
	mNode = node;
}

void RigidBodyState::setOffset(const Ogre::Vector3& offset)
{
	mCenterOfMassOffset.setOrigin(Convert::toBullet(offset));
}

void RigidBodyState::setOffset(const btVector3& offset)
{
	mCenterOfMassOffset.setOrigin(offset);
}

btVector3 RigidBodyState::getOffset() const
{
	return mCenterOfMassOffset.getOrigin();
}
