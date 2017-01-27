#pragma once
#include <G3D/G3DAll.h>

/** */
class RayTracer {
    protected:
         shared_ptr<TriTree> m_triangles;
         float m_numRays;
         bool m_isMultiThreaded;
         bool m_hasFixedPrimitives;
    public:
        RayTracer(const shared_ptr<Scene>& scene, const bool& isMultiThreaded, const float& numRays, const bool& hasFixedPrimitives);
        ~RayTracer();
        Radiance3 measureLight(const shared_ptr<Scene>& scene, const Ray& ray, int numScatters);
        bool findIntersection(shared_ptr<Surfel>& surfel, const Ray& ray);
        bool findSphereIntersection(const Ray& ray, const Point3 center, const float radius, float& t);
        bool findTriangleIntersection(const Ray& ray, const Tri& triangle, const CPUVertexArray& vertices, float& t, float b[3], TriTree::Hit& hit);
        void rayTrace(const shared_ptr<Scene>& scene, const shared_ptr<Camera>& cam, const shared_ptr<Image>& image);
        Radiance3 shade(const Ray& ray, const shared_ptr<Surfel>& surfel, const shared_ptr<Scene>& scene);
        bool isVisible(const Point3& X, const Point3& Y);
        Radiance3 colorSky(const Ray& ray,  const Point2int32& location);
};