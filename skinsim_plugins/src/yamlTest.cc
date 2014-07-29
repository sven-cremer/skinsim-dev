#include "yaml-cpp/yaml.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// our data types
struct Vec3 {
   float x, y, z;
};

struct Power {
   std::string name;
   int damage;
};

struct Monster {
   std::string name;
   Vec3 position;
   std::vector <Power> powers;
};

// Specs of the model to be built
struct ModelSpec
{
  // No of elements per side
  // do nothing if 0 use size_x and size_y
  double xByX         ; // : 5.0 # 0.0

  double d_pos        ; // : 0.0025 #0.005
  double density      ; // : 1.0

  double size_x       ; // : 0.0525
  double size_y       ; // : 0.0525

  double skin_height  ; // : 0.04
  double tac_height   ; // : 0.03

  double plane_height ; // : 0.02

  double sens_rad     ; // : 1.0
  double space_wid    ; // : 4.0
};

struct BuildModelSpec
{
  std::string name;
  ModelSpec        spec;
};


// now the extraction operators for these types
void operator >> (const YAML::Node& node, Vec3& v) {
   node[0] >> v.x;
   node[1] >> v.y;
   node[2] >> v.z;
}

void operator >> (const YAML::Node& node, Power& power) {
   node["name"] >> power.name;
   node["damage"] >> power.damage;
}

void operator >> (const YAML::Node& node, Monster& monster) {
   node["name"] >> monster.name;
   node["position"] >> monster.position;
   const YAML::Node& powers = node["powers"];
   for(unsigned i=0;i<powers.size();i++) {
      Power power;
      powers[i] >> power;
      monster.powers.push_back(power);
   }
}

// operators for emitting
YAML::Emitter& operator << (YAML::Emitter& out, const Vec3& v)
{
    out << YAML::Flow;
    out << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const Power& power)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << power.name;
    out << YAML::Key << "damage";
    out << YAML::Value << power.damage;
    out << YAML::EndMap;
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const Monster& monster)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << monster.name;
    out << YAML::Key << "position";
    out << YAML::Value << monster.position;
    out << YAML::Key << "powers";
    out << YAML::Value;
    out << YAML::BeginSeq;
    for(size_t pidx = 0; pidx < monster.powers.size(); ++pidx)
    {
        out << monster.powers[pidx];
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    return out;
}

// New stuff
void operator >> (const YAML::Node& node, ModelSpec& spec)
{
  node["xByX"        ] >> spec.xByX        ;
  node["d_pos"       ] >> spec.d_pos       ;
  node["density"     ] >> spec.density     ;
  node["size_x"      ] >> spec.size_x      ;
  node["size_y"      ] >> spec.size_y      ;
  node["skin_height" ] >> spec.skin_height ;
  node["tac_height"  ] >> spec.tac_height  ;
  node["plane_height"] >> spec.plane_height;
  node["sens_rad"    ] >> spec.sens_rad    ;
  node["space_wid"   ] >> spec.space_wid   ;
}

void operator >> (const YAML::Node& node, BuildModelSpec& buildModelSpec)
{
   node["name"] >> buildModelSpec.name;
   const YAML::Node& specs = node["spec"];
   ModelSpec spec;
   specs[0] >> spec;
   buildModelSpec.spec = spec;
}

YAML::Emitter& operator << (YAML::Emitter& out, const ModelSpec& spec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "xByX"        ; out << YAML::Value <<  spec.xByX        ;
    out << YAML::Key << "d_pos"       ; out << YAML::Value <<  spec.d_pos       ;
    out << YAML::Key << "density"     ; out << YAML::Value <<  spec.density     ;
    out << YAML::Key << "size_x"      ; out << YAML::Value <<  spec.size_x      ;
    out << YAML::Key << "size_y"      ; out << YAML::Value <<  spec.size_y      ;
    out << YAML::Key << "skin_height" ; out << YAML::Value <<  spec.skin_height ;
    out << YAML::Key << "tac_height"  ; out << YAML::Value <<  spec.tac_height  ;
    out << YAML::Key << "plane_height"; out << YAML::Value <<  spec.plane_height;
    out << YAML::Key << "sens_rad"    ; out << YAML::Value <<  spec.sens_rad    ;
    out << YAML::Key << "space_wid"   ; out << YAML::Value <<  spec.space_wid   ;
    out << YAML::EndMap;
    return out;
}

YAML::Emitter& operator << (YAML::Emitter& out, const BuildModelSpec& buildModelSpec)
{
    out << YAML::BeginMap;
    out << YAML::Key << "name";
    out << YAML::Value << buildModelSpec.name;
    out << YAML::Key << "spec";
    out << YAML::Value;

    out << YAML::BeginSeq;
    out << buildModelSpec.spec;
    out << YAML::EndSeq;

    out << YAML::EndMap;
    return out;
}

int main()
{

  // Write YAML files
  std::ofstream fout("test.yml");

  YAML::Emitter out;

  // Read YAML files
  std::ifstream fin("models.yaml");
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  out << YAML::BeginSeq;
  for(unsigned i=0;i<doc.size();i++)
  {
    BuildModelSpec modelSpecs;

//    modelSpecs.name = "spring_array";
//    modelSpecs.spec.xByX         = 5.0    ;
//    modelSpecs.spec.d_pos        = 0.0025 ;
//    modelSpecs.spec.density      = 1.0    ;
//    modelSpecs.spec.size_x       = 0.0525 ;
//    modelSpecs.spec.size_y       = 0.0525 ;
//    modelSpecs.spec.skin_height  = 0.04   ;
//    modelSpecs.spec.tac_height   = 0.03   ;
//    modelSpecs.spec.plane_height = 0.02   ;
//    modelSpecs.spec.sens_rad     = 1.0    ;
//    modelSpecs.spec.space_wid    = 4.0    ;

     doc[i] >> modelSpecs;
     std::cout << modelSpecs.name << "\n";

     out << modelSpecs;
  }
  out << YAML::EndSeq;

  fout << out.c_str();;
  fout.close();

  /*
  // Write YAML files
  std::ofstream fout("test.yml");

  YAML::Emitter out;

  // Read YAML files
  std::ifstream fin("monsters.yaml");
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  out << YAML::BeginSeq;
  for(unsigned i=0;i<doc.size();i++)
  {
     Monster monster;
     doc[i] >> monster;
     std::cout << monster.name << "\n";

     out << monster;
  }
  out << YAML::EndSeq;

  fout << out.c_str();;
  fout.close();
  */

  return 0;
}
