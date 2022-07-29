//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir = {0,0,0}, L_indir = {0,0,0};
    Intersection intersection = Scene::intersect(ray);
    if(!intersection.happened){
        return {};
    }
    //直接打到光源了
    if(intersection.m->hasEmission()){
        return intersection.m->getEmission();
    }

    Intersection lightpos;
    float lightpdf = 0.0f;
    sampleLight(lightpos,lightpdf);
    Vector3f collisionLight = lightpos.coords - intersection.coords;
    float dis = dotProduct(collisionLight,collisionLight);
    Vector3f collisionLightdir = collisionLight.normalized();
    Ray light_to_obj_ray(intersection.coords,collisionLightdir);
    Intersection light_to_anything_ray = Scene::intersect(light_to_obj_ray);
    auto f_r = intersection.m->eval(ray.direction,collisionLightdir,intersection.normal);
    if(light_to_anything_ray.distance - collisionLight.norm() > -0.005){
        L_dir = lightpos.emit * f_r * dotProduct(collisionLightdir,intersection.normal) * dotProduct(-collisionLightdir,lightpos.normal)/dis/lightpdf;
    }
    if(get_random_float() > RussianRoulette){
        return L_dir;
    }
    Vector3f w0 = intersection.m->sample(ray.direction,intersection.normal).normalized();
    Ray obj_to_obj_ray(intersection.coords,w0);
    Intersection isLight = Scene::intersect(obj_to_obj_ray);
    if(isLight.happened && !isLight.m->hasEmission()){
        float pdf = intersection.m->pdf(ray.direction,w0,intersection.normal);
        f_r = intersection.m->eval(ray.direction,w0,intersection.normal);
        L_indir = castRay(obj_to_obj_ray,depth + 1) * f_r * dotProduct(w0,intersection.normal)/pdf/RussianRoulette;
    }
    return L_dir + L_indir;
}