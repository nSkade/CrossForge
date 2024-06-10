#include "pngImport.h"
#include "color-util/RGB_to_XYZ.hpp"
#include "color-util/XYZ_to_Lab.hpp"
#include "color-util/CIEDE2000.hpp"
#include <crossforge/Utility/CForgeUtility.h>

#include <iostream>
#include <unordered_set>


namespace CForge{
    // png from top to bottom 

    // https://www.perplexity.ai/search/I-have-a-qUjvK2cGSl6pJAYneVZYiA
    void PNGImport::readPNGFile(const std::string &fileName){
        // check if fileName ends on png
        const std::string png = std::string(".png");

        bool is_png = fileName.substr(fileName.length() - png.length()) == png;  
        if(!is_png){
            throw CForgeExcept("Does not end with .png"); 
        }

        const char *cstr = fileName.c_str(); 
        FILE* fp = fopen(cstr, "rb"); 

        if(!fp){
            throw CForgeExcept("Failed to open file"); 
            return; 
        }

        png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        png_infop info_ptr = png_create_info_struct(png_ptr);
        
        png_init_io(png_ptr, fp);
        png_read_info(png_ptr, info_ptr);

        m_width = png_get_image_width(png_ptr, info_ptr);
        m_height = png_get_image_height(png_ptr, info_ptr);
        int color_type = png_get_color_type(png_ptr, info_ptr);
        int bit_depth = png_get_bit_depth(png_ptr, info_ptr); 

        std::vector<png_bytep> row_pointers(m_height);
        for (int y = 0; y < m_height; ++y) {
            row_pointers[y] = new png_byte[png_get_rowbytes(png_ptr, info_ptr)];
        }

        png_read_image(png_ptr, &row_pointers[0]);

        // Access pixel values here
        for (int y = 0; y < m_height; ++y) {
            png_bytep row = row_pointers[y];
            std::vector<Eigen::Vector4d> rowPixels; 
            for (int x = 0; x < m_width; ++x) {
                png_bytep px = &(row[x * 4]);
                // std::cout << "Pixel at (" << x << "," << y << ") has RGBA values: "
                //         << (int)px[0] << "," << (int)px[1] << "," << (int)px[2] << "," << (int)px[3] << std::endl;
                
                Eigen::Vector4d pixel = Eigen::Vector4d((int)px[0], (int)px[1], (int)px[2], (int)px[3]);
                rowPixels.push_back(pixel);  
            }
            m_pixelValues.push_back(rowPixels); 
        }

        for (int y = 0; y < m_height; ++y) {
            delete[] row_pointers[y];
        }
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp); 
    } 

    Eigen::Vector4d PNGImport::getPixel(int x, int y){
        // x is width, y is height
        if(m_pixelValues.empty()) return Eigen::Vector4d(0, 0, 0, 0); 
        if(x > m_width || x < 0 || y > m_height || y < 0){
            throw CForgeExcept("Index out of range"); 
        }

        return m_pixelValues.at(y).at(x); 
    } 

    Eigen::Vector4d PNGImport::uvCoordColor(float x, float y){
        // uv are organized from bottom to top, so oppiste to png
        // tranfrom to pixel coordinates
        int x_pixel = float(m_width) * x;  
        int y_pixel = m_height - int(float(m_height) * y); 

        return getPixel(x_pixel, y_pixel); 
    }

    bool PNGImport::isPictureLoaded(){
        return !m_pixelValues.empty();
    }

    Eigen::Vector4d PNGImport::getMeanColor(T3DMesh<float> *pMesh, int submeshIndex ,std::vector<int> *faceIndices, T3DMesh<float> *pOrigin, std::vector<int32_t> *correspondencyNotInverted){
        if(!isPictureLoaded()) throw CForgeExcept("No picture loaded"); 
        if(pMesh == nullptr || submeshIndex < 0 || submeshIndex >= pMesh->submeshCount()) throw CForgeExcept("Invalid mesh or submesh index");
        if(faceIndices == nullptr) throw CForgeExcept("No face indices provided");

        Eigen::Vector4d Rval;

        T3DMesh<float>::Submesh* pSubmesh = pMesh->getSubmesh(submeshIndex);
        std::unordered_set<int32_t> vertexIndicesSet;

        for(int i = 0; i < faceIndices->size(); i++){
            T3DMesh<float>::Face f = pSubmesh->Faces.at(faceIndices->at(i));
            for(int vertex = 0; vertex < 3; vertex++){
                vertexIndicesSet.insert(f.Vertices[vertex]);
            }
        }

        ///// get mean color - we need to use the corresponding vertices from the original mesh
        
        for(auto &vertex : vertexIndicesSet){ 
            int t = correspondencyNotInverted->at(vertex); 
            Eigen::Vector3f vt = pOrigin->textureCoordinate(t);
            Eigen::Vector4d color = uvCoordColor(vt[0], vt[1]);

            Rval += color; 
        }

        Rval /= vertexIndicesSet.size();

        return Rval; 
    }

    double ColorSpace::differenceRGBValues(Eigen::Vector3f rgb1, Eigen::Vector3f rgb2){
        colorutil::RGB rgb_color_1(rgb1[0] / 255.0, rgb1[1] / 255.0, rgb1[2] / 255.0);
        colorutil::RGB rgb_color_2(rgb2[0] / 255.0, rgb2[1] / 255.0, rgb2[2] / 255.0);

        colorutil::XYZ xyz_color_1 = colorutil::convert_RGB_to_XYZ(rgb_color_1);
        colorutil::XYZ xyz_color_2 = colorutil::convert_RGB_to_XYZ(rgb_color_2);
        colorutil::Lab lab_color_1 = colorutil::convert_XYZ_to_Lab(xyz_color_1);
        colorutil::Lab lab_color_2 = colorutil::convert_XYZ_to_Lab(xyz_color_2);

        double difference = colorutil::calculate_CIEDE2000(lab_color_1, lab_color_2);
        return difference; 
    }

} // namespace CForge