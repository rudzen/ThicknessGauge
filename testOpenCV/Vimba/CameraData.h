//#pragma once
//#include <string>
//#include <VimbaCPP/Include/Camera.h>
//#include <ostream>
//#include "../namespaces/tg.h"
//
//// contains all meta information from the current camera
//// reason : this is generic, the original vimba is not !
//class CameraData {
//
//    std::vector<std::string> Id;
//    std::vector<std::string> name;
//    std::vector<std::string> model;
//    std::vector<std::string> sn;
//    std::vector<std::string> interfaceId;
//
//public:
//
//    const char* getId() {
//        return Id.front().c_str();
//    }
//
//    const char* getName() {
//        return name.front().c_str();
//    }
//
//    const char* getModel() {
//        return model.front().c_str();
//    }
//
//    const char* getSn() {
//        return sn.front().c_str();
//    }
//
//    const char *getInterfaceId() {
//        return interfaceId.front().c_str();
//    }
//
//
//    bool empty() const {
//        return Id.empty();
//    }
//
//    size_t size() const {
//        return Id.size();
//    }
//
//    void clearAll() {
//        Id.clear();
//    }
//
//    void parse(AVT::VmbAPI::CameraPtrVector& cameras) {
//        std::string Id;
//        std::string name;
//        std::string model;
//        std::string sn;
//        std::string interfaceId;
//
//        auto size = cameras.size();
//
//        this->Id.reserve(size);
//        this->name.reserve(size);
//        this->model.reserve(size);
//        this->sn.reserve(size);
//        this->interfaceId.reserve(size);
//
//        for (auto& c : cameras) {
//            c->GetID(Id);
//            c->GetName(name);
//            c->GetModel(model);
//            c->GetSerialNumber(sn);
//            c->GetInterfaceID(interfaceId);
//            this->Id.emplace_back(Id);
//            this->name.emplace_back(name);
//            this->model.emplace_back(model);
//            this->sn.emplace_back(sn);
//            this->interfaceId.emplace_back(interfaceId);
//        }
//
//        this->Id.shrink_to_fit();
//        this->name.shrink_to_fit();
//        this->model.shrink_to_fit();
//        this->sn.shrink_to_fit();
//        this->interfaceId.shrink_to_fit();
//
//    }
//
//
//    friend std::ostream& operator<<(std::ostream& os, const CameraData& obj) {
//        auto size = obj.Id.size();
//
//        if (size == 0)
//            return os << "No cameras";
//
//        for (auto i = 0; i < size; i++) {
//            os << "\nCamera:\n";
//            os << "Id = " << obj.Id[i] << '\n';
//            os << "Name = " << obj.name[i] << '\n';
//            os << "Model = " << obj.model[i] << '\n';
//            os << "S/N = " << obj.sn[i] << '\n';
//            os << "InterfaceID = " << obj.interfaceId[i] << "\n";
//        }
//        return os;
//    }
//};
