// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CastResult.h>

JPH_NAMESPACE_BEGIN

bool NarrowPhaseQuery::CastRay(const RRayCast &inRay, RayCastResult &ioHit, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter) const
{
	JPH_PROFILE_FUNCTION();

	class MyCollector : public RayCastBodyCollector
	{
	public:
							MyCollector(const RRayCast &inRay, RayCastResult &ioHit, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter) :
			mRay(inRay),
			mHit(ioHit),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter)
		{
			UpdateEarlyOutFraction(ioHit.mFraction);
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			JPH_ASSERT(inResult.mFraction < mHit.mFraction, "This hit should not have been passed on to the collector");

			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult.mBodyID))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult.mBodyID);
				if (lock.Succeeded())
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						if (ts.CastRay(mRay, mHit))
						{
							// Test that we didn't find a further hit by accident
							JPH_ASSERT(mHit.mFraction >= C0 && mHit.mFraction < GetEarlyOutFraction());

							// Update early out fraction based on narrow phase collector
							UpdateEarlyOutFraction(mHit.mFraction);
						}
					}
				}
			}
		}

		RRayCast					mRay;
		RayCastResult &				mHit;
		const BodyLockInterface &	mBodyLockInterface;
		const BodyFilter &			mBodyFilter;
	};
	
	// Do broadphase test, note that the broadphase uses decimals so we drop precision here
	MyCollector collector(inRay, ioHit, *mBodyLockInterface, inBodyFilter);
	mBroadPhase->CastRay(RayCast(inRay), collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
	return ioHit.mFraction <= C1;
}

void NarrowPhaseQuery::CastRay(const RRayCast &inRay, const RayCastSettings &inRayCastSettings, CastRayCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	JPH_PROFILE_FUNCTION();

	class MyCollector : public RayCastBodyCollector
	{
	public:
							MyCollector(const RRayCast &inRay, const RayCastSettings &inRayCastSettings, CastRayCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			mRay(inRay),
			mRayCastSettings(inRayCastSettings),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
			UpdateEarlyOutFraction(ioCollector.GetEarlyOutFraction());
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			JPH_ASSERT(inResult.mFraction < mCollector.GetEarlyOutFraction(), "This hit should not have been passed on to the collector");

			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult.mBodyID))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult.mBodyID);
				if (lock.Succeeded())
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CastRay(mRay, mRayCastSettings, mCollector, mShapeFilter);

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		RRayCast					mRay;
		RayCastSettings				mRayCastSettings;
		CastRayCollector &			mCollector;
		const BodyLockInterface &	mBodyLockInterface;
		const BodyFilter &			mBodyFilter;
		const ShapeFilter &			mShapeFilter;
	};

	// Do broadphase test, note that the broadphase uses decimals so we drop precision here
	MyCollector collector(inRay, inRayCastSettings, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhase->CastRay(RayCast(inRay), collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollidePoint(RVec3Arg inPoint, CollidePointCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	JPH_PROFILE_FUNCTION();

	class MyCollector : public CollideShapeBodyCollector
	{
	public:
							MyCollector(RVec3Arg inPoint, CollidePointCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			mPoint(inPoint),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult);
				if (lock.Succeeded())
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CollidePoint(mPoint, mCollector, mShapeFilter);

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		RVec3							mPoint;
		CollidePointCollector &			mCollector;
		const BodyLockInterface &		mBodyLockInterface;
		const BodyFilter &				mBodyFilter;
		const ShapeFilter &				mShapeFilter;
	};

	// Do broadphase test (note: truncates double to single precision since the broadphase uses single precision)
	MyCollector collector(inPoint, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhase->CollidePoint(Vec3(inPoint), collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollideShape(const Shape *inShape, Vec3Arg inShapeScale, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings &inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	JPH_PROFILE_FUNCTION();

	class MyCollector : public CollideShapeBodyCollector
	{
	public:
							MyCollector(const Shape *inShape, Vec3Arg inShapeScale, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings &inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			mShape(inShape),
			mShapeScale(inShapeScale),
			mCenterOfMassTransform(inCenterOfMassTransform),
			mCollideShapeSettings(inCollideShapeSettings),
			mBaseOffset(inBaseOffset),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult);
				if (lock.Succeeded())
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CollideShape(mShape, mShapeScale, mCenterOfMassTransform, mCollideShapeSettings, mBaseOffset, mCollector, mShapeFilter);

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		const Shape *					mShape;
		Vec3							mShapeScale;
		RMat44							mCenterOfMassTransform;
		const CollideShapeSettings &	mCollideShapeSettings;
		RVec3							mBaseOffset;
		CollideShapeCollector &			mCollector;
		const BodyLockInterface &		mBodyLockInterface;
		const BodyFilter &				mBodyFilter;
		const ShapeFilter &				mShapeFilter;
	};

	// Calculate bounds for shape and expand by max separation distance
	AABox bounds = inShape->GetWorldSpaceBounds(inCenterOfMassTransform, inShapeScale);
	bounds.ExpandBy(Vec3::sReplicate(inCollideShapeSettings.mMaxSeparationDistance));

	// Do broadphase test
	MyCollector collector(inShape, inShapeScale, inCenterOfMassTransform, inCollideShapeSettings, inBaseOffset, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhase->CollideAABox(bounds, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CastShape(const RShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	JPH_PROFILE_FUNCTION();

	class MyCollector : public CastShapeBodyCollector
	{
	private:
			/// Update early out fraction based on narrow phase collector
			inline void		PropagateEarlyOutFraction()
			{
				// The CastShapeCollector uses negative values for penetration depth so we want to clamp to the smallest positive number to keep receiving deeper hits
				if (mCollector.ShouldEarlyOut())
					ForceEarlyOut();
				else
					UpdateEarlyOutFraction(max(FIX_MIN, mCollector.GetEarlyOutFraction()));
			}

	public:
							MyCollector(const RShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			mShapeCast(inShapeCast),
			mShapeCastSettings(inShapeCastSettings),
			mBaseOffset(inBaseOffset),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
			PropagateEarlyOutFraction();
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			JPH_ASSERT(inResult.mFraction <= max(C0, mCollector.GetEarlyOutFraction()), "This hit should not have been passed on to the collector");

			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult.mBodyID))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult.mBodyID);
				if (lock.Succeeded())
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CastShape(mShapeCast, mShapeCastSettings, mBaseOffset, mCollector, mShapeFilter);

						// Update early out fraction based on narrow phase collector
						PropagateEarlyOutFraction();
					}
				}
			}
		}

		RShapeCast					mShapeCast;
		const ShapeCastSettings &	mShapeCastSettings;
		RVec3						mBaseOffset;
		CastShapeCollector &		mCollector;
		const BodyLockInterface &	mBodyLockInterface;
		const BodyFilter &			mBodyFilter;
		const ShapeFilter &			mShapeFilter;
	};

	// Do broadphase test
	MyCollector collector(inShapeCast, inShapeCastSettings, inBaseOffset, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhase->CastAABox({ inShapeCast.mShapeWorldBounds, inShapeCast.mDirection }, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollectTransformedShapes(const AABox &inBox, TransformedShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	class MyCollector : public CollideShapeBodyCollector
	{
	public:
							MyCollector(const AABox &inBox, TransformedShapeCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			mBox(inBox),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult);
				if (lock.Succeeded())
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CollectTransformedShapes(mBox, mCollector, mShapeFilter);

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		const AABox &					mBox;
		TransformedShapeCollector &		mCollector;
		const BodyLockInterface &		mBodyLockInterface;
		const BodyFilter &				mBodyFilter;
		const ShapeFilter &				mShapeFilter;
	};

	// Do broadphase test
	MyCollector collector(inBox, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhase->CollideAABox(inBox, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

JPH_NAMESPACE_END
