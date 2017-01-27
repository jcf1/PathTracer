/** \file RayTracer.cpp */
#include "RayTracer.h"

// Iterates through each pixel, creating a ray for each one from the pixel to the camera aperature, and using measureLight to find the radiance for each pixel
void RayTracer::rayTrace(const shared_ptr<Scene>& scene, const shared_ptr<Camera>& cam, const shared_ptr<Image>& image){
    float imWidth = image->width();
    float imHeight = image->height();
    Rect2D rect2D(Vector2(imWidth-1, imHeight-1));
    if(!m_isMultiThreaded){
        //Now iterate through all of the pixels
        for(int y(0); y < image->height();++y){
            for(int x(0); x<image->width();++x){
                Ray ray = cam->worldRay(x+.5f,y+.5f,rect2D);
                Radiance3 radiance = measureLight(scene, ray, 2);
                if(radiance.r == 0 && radiance.g == 0 && radiance.b == 0) radiance = colorSky(ray, Point2int32(x,y));
                image->set(Point2int32(x,y), radiance);
            }
        }
    } else {
        Thread::runConcurrently(Point2int32(0,0), Point2int32(image->width(), image->height()), [this, scene, cam, image, rect2D](Point2int32& vertex) -> void {
            Ray ray = cam->worldRay(vertex.x +0.5f, vertex.y+0.5f, rect2D);
            Radiance3 radiance = measureLight(scene,ray,2);
            if(radiance.r == 0 && radiance.g == 0 && radiance.b == 0) radiance = colorSky(ray, vertex);
            image->set(vertex, radiance);
        }); 
    }


}

Radiance3 RayTracer::colorSky(const Ray& ray, const Point2int32& location){
    Point3 P(ray.origin());
    Point2int32 p(P.x,P.y);
    Radiance3 picCol(0.0f,0.75f,1.0f);
    float dis(sqrt(abs((p.x-location.x)*(p.x-location.x)+(p.y-location.y)*(p.y-location.y))));
    picCol *= (dis>1) ? abs( 1/dis * 100 * ray.direction().dot(Vector3(0.04f,0.5f,0.03f))) : abs(dis * ray.direction().dot(Vector3(0.04f,0.5f,0.03f)) * 100);
    return picCol;
}

// Will calculate the radiance for a single ray of light by finding the intersection and recursively 
// scattering light
Radiance3 RayTracer::measureLight(const shared_ptr<Scene>& scene, const Ray& ray, int numScatters){
    if (numScatters == 0){
        return Radiance3(0,0,0);
    }
        //shared_ptr<Surfel> surfel = triangles.intersectRay(ray);
    shared_ptr<Surfel> surfel;
    bool doesIntersect = findIntersection(surfel, ray);

    Radiance3 returnRadiance(0,0,0);
    if (!doesIntersect){
        return Radiance3(0,0,0);
    }
    returnRadiance  = shade(ray, surfel, scene);
    for (int i(0); i < m_numRays; ++i){
        Point3 check = surfel->position;
        Ray otherRay(surfel->position + FLT_EPSILON*surfel->geometricNormal, Vector3::hemiRandom(surfel->shadingNormal));
        int num = numScatters - 1;
        returnRadiance += (measureLight(scene, otherRay, num)/m_numRays);
    }
    return returnRadiance;
    
    // if (findIntersection(surfel, ray, triangles, vertices)) return Radiance3(1,1,1);
    //else return Radiance3(0,0,0);
}   


/*Will iterate through a TriTree, calling findTriangleIntersection for each Tri. If there are primitives, it will call findSphereIntersection for them. 
It should use the surfel it hits first (the one with the shortest distance from the camera.)*/
bool RayTracer::findIntersection(shared_ptr<Surfel>& surfel, const Ray& ray){
    Point3 P = ray.origin();
    Vector3 w = ray.direction();
    
    float bar[3];
    float b[3];
    float t;
    float min = INFINITY;
    TriTree::Hit tempHit = TriTree::Hit();
    TriTree::Hit hit = TriTree::Hit();

    for (int i(0); i < m_triangles->size(); ++i) {
        bool intersects = findTriangleIntersection(ray, m_triangles->operator[](i), m_triangles->vertexArray(), t, b, tempHit);
        if (intersects){
            if (t < min){
                min = t;
                bar[0] = b[0];
                bar[1] = b[1];
                bar[2] = b[2];
                hit = tempHit;
                hit.triIndex = i;
                hit.distance = t;
                hit.u = b[0];
                hit.v = b[1];
;            }
        }
    }

    bool isCircleCloser = false;
    
    if (m_hasFixedPrimitives){
        Point3 center(-2.5, 1.5, -.5f);
        float radius(.3);
        for(int i(0); i < 2; ++i){
            center +=  i*Vector3(1,0,0);
            bool intersects = findSphereIntersection(ray, center + Vector3(2,0,0), radius, t);
            if(intersects){
                if (t<min){ 
                    isCircleCloser = true;
                    min = t;
                    auto surf = std::make_shared<UniversalSurfel>();
                    surf->name = "analytical";
                    surf->position = Point3(ray.origin() + t*ray.direction());
                    surf->lambertianReflectivity = Color3(1.46f,.5f,1.47f);
                    surf->shadingNormal = (surf->position-center).unit();
                    surf->emission = Radiance3(0.0f,0.0f,0.0f);
                    surfel = surf;
                }
            }
        }
    }

    if (min == INFINITY){
        return false;
    }
    
    if (!isCircleCloser) surfel = m_triangles->sample(hit);

    //return m_triangles->operator[](hit.triIndex).intersectionAlphaTest(m_triangles->vertexArray(), hit.u, hit.v, 1.0f);
    
    return true;
    
}


// Find the interesection of a ray to a sphere.
bool RayTracer::findSphereIntersection(const Ray& ray, const Point3 center, const float radius, float& t){
    Point3 P = ray.origin();
    Vector3 w = ray.direction();

    float a = 1;
    float b = 2*w.dot(P-center);
    float c = ((P-center).dot(P-center) - radius * radius);

    if((b*b-4*a*c)<0) return false;
    
    float posT((-b + sqrt(b*b-4*a*c))/(2*a));
    float negT((-b + sqrt(b*b-4*a*c))/(2*a));
   
    if(posT<0 && negT< 0) return false;
    else if(posT<0) t = negT; 
    else if(negT<0) t = posT;
    else t = min(posT,negT);
    return true;
}

// find the intersection of a ray to a triangle
bool RayTracer::findTriangleIntersection(const Ray& ray, const Tri& triangle, const CPUVertexArray& vertices, float& t, float b[3], TriTree::Hit& hit){

    const Point3& P = ray.origin();
    const Vector3& w(ray.direction());
    const Vector3& e1(triangle.e1(vertices));
    const Vector3& e2(triangle.e2(vertices));
    const Vector3& n(triangle.normal(vertices));

    const Vector3& q(w.cross(e2));
    float a = e1.dot(q);

    if (n.dot(w) >= 0 || abs(a) <= 0.0001f) return false;

    const Vector3& s((P-triangle.vertex(vertices,0).position)/a);
    const Vector3& r = s.cross(e1);

    b[0] = s.dot(q);
    b[1] = r.dot(w);
    b[2] = 1.0f - b[0] - b[1];

    if(w.dot(n) > 0){
        hit.backface = true;
    }

    if(b[0] < 0.0f || b[1] < 0.0f || b[2] < 0.0f) return false;

    t = e2.dot(r);
    return (t >= 0.0f);

}

Radiance3 RayTracer::shade(const Ray& ray, const shared_ptr<Surfel>& surfel, const shared_ptr<Scene>& scene){
    Radiance3 L = surfel->reflectivity(Random::threadCommon())*0.05f; //surfel->emittedRadiance(-1*ray.direction());
    const Point3& X = surfel->position;
    const Vector3& n = surfel->shadingNormal;
    Array<shared_ptr<Light>> lights = scene->lightingEnvironment().lightArray;
    for(int i = 0; i < lights.size(); ++i) {
        const shared_ptr<Light> light(lights[i]);
        const Point3& Y = light->position().xyz();

        if (!light->castsShadows() || isVisible(X,Y)){
            const Vector3& w_i = (Y-X).direction();
            Biradiance3& Bi = light->biradiance(X);

            const Color3& f = surfel->finiteScatteringDensity(w_i,-1*ray.direction());
            L+=Bi * f * abs(w_i.dot(n));
        }
    }
    return L;
}

bool RayTracer::isVisible(const Point3& X, const Point3& Y){
    Point3 origin (Y + FLT_EPSILON*(X-Y));
    Vector3 direction = (X-Y).unit();
    Ray newRay(origin, direction);
    shared_ptr<Surfel> surfel;
    if(findIntersection(surfel, newRay))
        return (abs(surfel->position.x-X.x)<0.001f && abs(surfel->position.y-X.y) < 0.01f && abs(surfel->position.z - X.z) < 0.001f);
    return false;
}

RayTracer::RayTracer(const shared_ptr<Scene>& scene, const bool& isMultiThreaded, const float& numRays, const bool& hasFixedPrimitives){
    // Get all the surfaces in a scene
    Array<shared_ptr<Surface>> sceneSurfaces;
    scene->onPose(sceneSurfaces);
    
    // Get all the triangles in a scene from all the surfaces
    m_triangles = shared_ptr<TriTree>(new TriTree());
    m_triangles->setContents(sceneSurfaces);
    m_isMultiThreaded = isMultiThreaded;
    m_numRays = numRays;
    m_hasFixedPrimitives = hasFixedPrimitives;
}

RayTracer::~RayTracer(){
   // delete &m_triangles;
}

