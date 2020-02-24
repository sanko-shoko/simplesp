//--------------------------------------------------------------------------------
// Copyright (c) 2017-2019, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_DAE_H__
#define __SP_DAE_H__

#include "spapp/spdata/spfile.h"
#include "spapp/spdata/spmodel.h"
#include "spapp/spdata/spxml.h"
#include "spapp/spdata/sptga.h"

namespace sp{

    //--------------------------------------------------------------------------------
    // local method
    //--------------------------------------------------------------------------------
    
    namespace _tga {

        SP_CPUFUNC void library_images(XML &xml, const char *name) {
            XML_NEST(xml, "library_images");
            if (name != NULL && name[0] != 0) {
                char str[256];
                sprintf(str, "image id=\"Material_Diffuse_Color\" name=\"Material_Diffuse_Color\"");
                XML_NEST(xml, str);
                {
                    sprintf(str, "%s.png", name);
                    xml.add("init_from", str);
                }
            }
        }

        SP_CPUFUNC void library_effects(XML &xml) {
            XML_NEST(xml, "library_effects");
            {
                XML_NEST(xml, "effect id=\"Material-effect\"");
                {
                    XML_NEST(xml, "profile_COMMON");
                    {
                        XML_NEST(xml, "newparam sid=\"Material_Diffuse_Color-surface\"");
                        {
                            XML_NEST(xml, "surface type=\"2D\"");
                            xml.add("init_from", "Material_Diffuse_Color");
                        }
                    }
                    {
                        XML_NEST(xml, "newparam sid=\"Material_Diffuse_Color-sampler\"");
                        {
                            XML_NEST(xml, "sampler2D");
                            xml.add("source", "Material_Diffuse_Color-surface");
                        }
                    }

                    {
                        XML_NEST(xml, "technique sid=\"common\"");
                        {
                            XML_NEST(xml, "phong");
                            {
                                XML_NEST(xml, "emission");
                                xml.add("color sid=\"emission\"", "0 0 0 1");
                            }
                            {
                                XML_NEST(xml, "ambient");
                                xml.add("color sid=\"ambient\"", "0 0 0 1");
                            }
                            {
                                XML_NEST(xml, "diffuse");
                                xml.add("texture texture=\"Material_Diffuse_Color-sampler\"");
                            }
                            {
                                XML_NEST(xml, "specular");
                                xml.add("color sid=\"specular\"", "0.5 0.5 0.5 1");
                            }
                            {
                                XML_NEST(xml, "shininess");
                                xml.add("float sid=\"shininess\"", "50");
                            }
                            {
                                XML_NEST(xml, "index_of_refraction");
                                xml.add("float sid=\"index_of_refraction\"", "1");
                            }
                        }
                    }
                }

            }
        }

        SP_CPUFUNC void library_materials(XML &xml) {
            XML_NEST(xml, "library_materials");
            {
                XML_NEST(xml, "material id=\"Material-material\" name=\"Material\"");
                {
                    XML_NEST(xml, "instance_effect url=\"#Material-effect\"");

                }
            }
        }

        SP_CPUFUNC void library_geometries(XML &xml, const Mem1<Vec3> &vtxs, const Mem1<Vec3> &nrms, const Mem1<Col3> &cols, const Mem1<int> &idxs) {
            XML_NEST(xml, "library_geometries");
            {
                XML_NEST(xml, "geometry id=\"model-mesh\" name=\"model\"");
                {
                    XML_NEST(xml, "mesh");
                    {
                        XML_NEST(xml, "source id=\"model-mesh-positions\"");
                        {
                            std::string val;
                            {
                                char str[256];
                                for (int i = 0; i < vtxs.size(); i++) {
                                    sprintf(str, "%f %f %f", (float)vtxs[i].x, (float)vtxs[i].y, (float)vtxs[i].z);
                                    val += str;
                                    if (i < vtxs.size() - 1) val += " ";
                                }
                            }
                            {
                                char str[256];
                                sprintf(str, "float_array id=\"model-mesh-positions-array\" count=\"%d\"", vtxs.size() * 3);
                                xml.add(str, val.c_str());
                            }

                            XML_NEST(xml, "technique_common");
                            {
                                char str[256];
                                sprintf(str, "accessor source=\"#model-mesh-positions-array\" count=\"%d\" stride = \"3\"", vtxs.size());

                                XML_NEST(xml, str);
                                {
                                    xml.add("param name=\"X\" type=\"float\"", NULL);
                                    xml.add("param name=\"Y\" type=\"float\"", NULL);
                                    xml.add("param name=\"Z\" type=\"float\"", NULL);
                                }
                            }
                        }
                    }
                    {
                        XML_NEST(xml, "source id=\"model-mesh-normals\"");
                        {
                            std::string val;
                            {
                                char str[256];
                                for (int i = 0; i < nrms.size(); i++) {
                                    sprintf(str, "%f %f %f", (float)nrms[i].x, (float)nrms[i].y, (float)nrms[i].z);
                                    val += str;
                                    if (i < nrms.size() - 1) val += " ";
                                }
                            }
                            {
                                char str[256];
                                sprintf(str, "float_array id=\"model-mesh-normals-array\" count=\"%d\"", nrms.size() * 3);
                                xml.add(str, val.c_str());
                            }

                            XML_NEST(xml, "technique_common");
                            {
                                char str[256];
                                sprintf(str, "accessor source = \"#model-mesh-normals-array\" count=\"%d\" stride=\"3\"", nrms.size());

                                XML_NEST(xml, str);
                                {
                                    xml.add("param name=\"X\" type=\"float\"", NULL);
                                    xml.add("param name=\"Y\" type=\"float\"", NULL);
                                    xml.add("param name=\"Z\" type=\"float\"", NULL);
                                }
                            }
                        }
                    }
                    {
                        XML_NEST(xml, "source id=\"model-mesh-map-0\"");
                        {
                            std::string val;
                            {
                                char str[256];
                                const int w = 64;
                                const int h = (cols.size() + 64 - 1) / w;

                                for (int i = 0; i < cols.size(); i++) {
                                    Col3f c = cast<Col3f>(cols[i]);

                                    const int u = i % 64;
                                    const int v = (h - 1) - i / 64;
                                    const float fu = static_cast<float>(u + 0.5) / (w);
                                    const float fv = static_cast<float>(v + 0.5) / (h);

                                    sprintf(str, "%f %f", fu, fv);
                                    val += str;
                                    if (i < cols.size() - 1) val += " ";
                                }
                            }
                            {
                                char str[256];
                                sprintf(str, "float_array id=\"model-mesh-map-0-array\" count=\"%d\"", cols.size() * 2);
                                xml.add(str, val.c_str());
                            }

                            XML_NEST(xml, "technique_common");
                            {
                                char str[256];
                                sprintf(str, "accessor source=\"#model-mesh-map-0-array\" count=\"%d\" stride=\"2\"", cols.size());

                                XML_NEST(xml, str);
                                {
                                    xml.add("param name=\"S\" type=\"float\"");
                                    xml.add("param name=\"T\" type=\"float\"");
                                }
                            }
                        }
                    }
                    {
                        XML_NEST(xml, "vertices id=\"model-mesh-vertices\"");
                        {
                            XML_NEST(xml, "input semantic=\"POSITION\" source=\"#model-mesh-positions\"");
                        }
                    }
                    {
                        char str[256];
                        sprintf(str, "triangles material=\"Material-material\" count=\"%d\"", idxs.size());
                        XML_NEST(xml, str);
                        {
                            xml.add("input semantic=\"VERTEX\" source=\"#model-mesh-vertices\" offset=\"0\"");
                            xml.add("input semantic=\"NORMAL\" source=\"#model-mesh-normals\" offset=\"1\"");
                            xml.add("input semantic=\"TEXCOORD\" source=\"#model-mesh-map-0\" offset=\"2\" set=\"0\"");
                        }
                        {
                            std::string val;
                            {
                                char str[256];
                                for (int i = 0; i < idxs.size(); i++) {
                                    sprintf(str, "%d", idxs[i]);
                                    val += str;
                                    if (i < idxs.size() - 1) val += " ";
                                }
                            }
                            xml.add("p", val.c_str());
                        }
                    }
                }
            }
        }

    }


    SP_CPUFUNC bool saveDAE(const char *path, const Mem1<Vec3> &vtxs, const Mem1<Vec3> &nrms, const Mem1<Col3> &cols, const Mem1<int> &idxs) {
        char tex[256] = { 0 };
        {
            const int w = 64;
            const int h = (cols.size() + 64 - 1) / w;

            Mem2<Col4> img(w, h);
            img.zero();
            for (int i = 0; i < cols.size(); i++) {
                img[i] = cast<Col4>(cols[i]);
            }

            const int size = strlen(path);
            for (int i = 0; i < size; i++) {
                if (path[i] == 0 || path[i] == '.') {
                    break;
                }
                tex[i] = path[i];
            }

            char _path[256] = { 0 };
            sprintf(_path, "%s.tga", tex);
            saveTGA(_path, img);
        }

        XML xml;
        
        {
            XML_NEST(xml, "COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"");
            {
                XML_NEST(xml, "asset");
                xml.add("unit name=\"meter\" meter=\"1\"", NULL);
                xml.add("up_axis", "Y_UP");
            }
            _tga::library_images(xml, tex);
            _tga::library_effects(xml);
            _tga::library_materials(xml);
            _tga::library_geometries(xml, vtxs, nrms, cols, idxs);

            {
                
            }
            {
                XML_NEST(xml, "library_controllers");
            }
            {
                XML_NEST(xml, "library_visual_scenes");
                {
                    XML_NEST(xml, "visual_scene id=\"Scene\" name=\"Scene\"");
                    {
                        XML_NEST(xml, "node id=\"model\" name=\"model\" type=\"NODE\"");
                        {
                            {
                                xml.add("matrix sid=\"transform\"", "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 0");
                            }
                            {
                                XML_NEST(xml, "instance_geometry url=\"#model-mesh\" name=\"model\"");

                                {
                                    XML_NEST(xml, "bind_material");
                                    {
                                        XML_NEST(xml, "technique_common");
                                        {
                                            xml.add("instance_material symbol=\"Material-material\" target=\"#Material-material\"");

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            {
                XML_NEST(xml, "scene");
                {
                    XML_NEST(xml, "instance_visual_scene url=\"#Scene\"");
                    {

                    }
                }
            }
        }

        xml.save(path);
        return true;
    }

}

#endif