#include "easy_image.h"
#include "ini_configuration.h"
#include "Lines2D.cpp"
#include "l_parser/l_parser.h"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>


img::EasyImage draw2DLines (const Lines2D &lines, const int size){
    double xmin{0};
    double xmax{0};
    double ymin{0};
    double ymax{0};
    for (auto line : lines){
        xmin = (line.p1.x < xmin) ? line.p1.x : xmin;
        xmin = (line.p2.x < xmin) ? line.p2.x : xmin;
        xmax = (line.p1.x > xmax) ? line.p1.x : xmax;
        xmax = (line.p2.x > xmax) ? line.p2.x : xmax;
        ymin = (line.p1.y < ymin) ? line.p1.y : xmin;
        ymin = (line.p2.y < ymin) ? line.p2.y : xmin;
        ymax = (line.p1.y > ymax) ? line.p1.y : xmax;
        ymax = (line.p2.y > ymax) ? line.p2.y : xmax;
    };

    double xrange = xmax - xmin;
    double yrange = ymax - ymin;
    double imagex = size * xrange / fmax(xrange, yrange);
    double imagey = size * yrange / fmax(xrange, yrange);
    double scale = 0.95 * imagex / xrange;

    for (auto line : lines){
        line.p1.x *= scale;
        line.p1.y *= scale;
        line.p2.x *= scale;
        line.p2.y *= scale;
    }

    return {};
}


Lines2D LSystem(const std::string &input, const Color &lineColor){
    LParser::LSystem2D l_system;
    std::ifstream input_stream(input);
    input_stream >> l_system;input_stream.close();

    std::set<char> alphabet = l_system.get_alphabet();
    std::string initiator = l_system.get_initiator();
    double angle = l_system.get_starting_angle();
    double currentAngle = l_system.get_starting_angle();
    unsigned int iterations = l_system.get_nr_iterations();

    for (int i=0 ; i<iterations ; i++){
        std::string newString{""};
        for (auto c : initiator){
            newString.append((alphabet.find(c) == alphabet.end()) ?
            std::to_string(c) : l_system.get_replacement(c));
        }
        initiator = newString;
    }

    Lines2D out;
    Point2D p1(0, 0);
    Point2D p2(0, 0);
    for (auto c : initiator){
        if (alphabet.find(c) == alphabet.end()){
            if (c == '-') currentAngle -= angle;
            else if (c = '+') currentAngle += angle;
        } else {
            p2.x += cos(currentAngle);
            p2.y += sin(currentAngle);
            if (l_system.draw(c)) out.push_back(Line2D(p1, p2, lineColor));
            p1.x = p2.x;
            p1.y = p2.y;
        }
    }
    return out;
}


img::EasyImage generate2DLinesImage(const ini::Configuration &conf){
    unsigned int size = conf["General"]["size"].as_int_or_die();
    std::vector<double> bgColor = conf["General"]["backgroundcolor"].as_double_tuple_or_die();
    std::string inputFile = conf["2DLSystem"]["inputfile"].as_string_or_die();
    std::vector<double> color = conf["2DLSystem"]["color"].as_double_tuple_or_die();
    img::EasyImage image;

    image = draw2DLines(LSystem(inputFile, Color(color[0],color[1],color[2])), size);

    return image;
}


img::EasyImage generate_image(const ini::Configuration &configuration) {
    img::EasyImage image;
    std::string type = configuration["General"]["type"].as_string_or_die();

    if (type == "2DLSystem") image = generate2DLinesImage(configuration);

	return image;
}


int main(int argc, char const* argv[])
{
        int retVal = 0;
        try
        {
                std::vector<std::string> args = std::vector<std::string>(argv+1, argv+argc);
                if (args.empty()) {
                        std::ifstream fileIn("filelist");
                        std::string filelistName;
                        while (std::getline(fileIn, filelistName)) {
                                args.push_back(filelistName);
                        }
                }
                for(std::string fileName : args)
                {
                        ini::Configuration conf;
                        try
                        {
                                std::ifstream fin(fileName);
                                if (fin.peek() == std::istream::traits_type::eof()) {
                                    std::cout << "Ini file appears empty. Does '" <<
                                    fileName << "' exist?" << std::endl;
                                    continue;
                                }
                                fin >> conf;
                                fin.close();
                        }
                        catch(ini::ParseException& ex)
                        {
                                std::cerr << "Error parsing file: " << fileName << ": " << ex.what() << std::endl;
                                retVal = 1;
                                continue;
                        }

                        img::EasyImage image = generate_image(conf);
                        if(image.get_height() > 0 && image.get_width() > 0)
                        {
                                std::string::size_type pos = fileName.rfind('.');
                                if(pos == std::string::npos)
                                {
                                        //filename does not contain a '.' --> append a '.bmp' suffix
                                        fileName += ".bmp";
                                }
                                else
                                {
                                        fileName = fileName.substr(0,pos) + ".bmp";
                                }
                                try
                                {
                                        std::ofstream f_out(fileName.c_str(),std::ios::trunc | std::ios::out | std::ios::binary);
                                        f_out << image;

                                }
                                catch(std::exception& ex)
                                {
                                        std::cerr << "Failed to write image to file: " << ex.what() << std::endl;
                                        retVal = 1;
                                }
                        }
                        else
                        {
                                std::cout << "Could not generate image for " << fileName << std::endl;
                        }
                }
        }
        catch(const std::bad_alloc &exception)
        {
    		//When you run out of memory this exception is thrown. When this happens the return value of the program MUST be '100'.
    		//Basically this return value tells our automated test scripts to run your engine on a pc with more memory.
    		//(Unless of course you are already consuming the maximum allowed amount of memory)
    		//If your engine does NOT adhere to this requirement you risk losing points because then our scripts will
		//mark the test as failed while in reality it just needed a bit more memory
                std::cerr << "Error: insufficient memory" << std::endl;
                retVal = 100;
        }
        return retVal;
}
