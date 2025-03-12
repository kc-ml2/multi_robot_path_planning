#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/pointer.h"
#include "rapidjson/rapidjson.h"

#include <string>
#include <iostream>
#include <fstream>

using namespace rapidjson;

int main(){

    std::string filename = "test.txt";

    Document d;
    d.SetObject();

    Document::AllocatorType& allocator = d.GetAllocator();

    size_t sz = allocator.Size();

    d.AddMember("map_file",  "hello world", d.GetAllocator());
    
    Value datas(kArrayType);
    // NO1 data
    Value data(kObjectType);
    data.AddMember("robot_id", 0, d.GetAllocator());

    std::cout << data["robot_id"].GetInt() << std::endl;

    Value histories(kArrayType);    
    Value history(kObjectType);

    //Node json
    history.AddMember("index", 0, d.GetAllocator());
    history.AddMember("parent", 0, d.GetAllocator());
    Value state(kArrayType);
    state.PushBack(0.1, d.GetAllocator());
    state.PushBack(0.2, d.GetAllocator());
    state.PushBack(0.3, d.GetAllocator());

    history.AddMember("state", state, d.GetAllocator());
    history.AddMember("costs", 0.0, d.GetAllocator());
    history.AddMember("timestamp", 0, d.GetAllocator());

    histories.PushBack(history, d.GetAllocator());

        //std::cout<<histories[0].GetDouble()<<std::endl;
    std::cout<<histories.Size()<<std::endl;

    int count = 0;

    for (auto& v : histories.GetArray()){
        for (auto& w : v["state"].GetArray()){
            std::cout<<w.GetDouble()<<std::endl;
        }
    }

    for (int i =  0; i < histories.Size(); i++)
    {
        count++;
    }
    std::cout<<count<<std::endl;








    data.AddMember("history", histories, d.GetAllocator());
    datas.PushBack(data,d.GetAllocator());
    //

    d.AddMember("data", datas, d.GetAllocator());
    







    Value* aa;
    aa = new Value(kArrayType);
    aa->PushBack(0, d.GetAllocator());
    aa->PushBack(1, d.GetAllocator());
    d.AddMember("aa", (*aa), d.GetAllocator());

    delete(aa);







    d.AddMember("data",   2, allocator);
    d.AddMember("group",    3, allocator);
    d.AddMember("order",    4, allocator);

    Value tests(kArrayType);
    Value obj(kObjectType);
    Value val(kObjectType);

    obj.AddMember("id", 1, allocator);

    std::string description = "a description";
    val.SetString(description.c_str(), static_cast<SizeType>(description.length()), allocator);
    obj.AddMember("description", val, allocator);

    std::string help = "some help";
    val.SetString(help.c_str(), static_cast<SizeType>(help.length()), allocator);
    obj.AddMember("help", val, allocator);

    std::string workgroup = "a workgroup";
    val.SetString(workgroup.c_str(), static_cast<SizeType>(workgroup.length()), allocator);
    obj.AddMember("workgroup", val, allocator);

    val.SetBool(true);
    obj.AddMember("online", val, allocator);

    tests.PushBack(obj, allocator);
    d.AddMember("tests", tests, allocator);

    // Convert JSON document to string
    rapidjson::StringBuffer strbuf;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(strbuf);
    d.Accept(writer);

    // Output {"project":"rapidjson","stars":11}
    //std::cout << strbuf.GetString() << std::endl;

    std::ofstream writeFile(filename.data());
    if(writeFile.is_open()){
        writeFile << strbuf.GetString() << std::endl;
        writeFile.close();
    }

    return 0;
}