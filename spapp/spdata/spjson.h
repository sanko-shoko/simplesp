﻿//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_JSON_H__
#define __SP_JSON_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spfile.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // json
    //--------------------------------------------------------------------------------

    class JSON {
    
    public:

        struct Unit {
            Unit *parent;
            Mem1<Unit*> child;

            std::string tag;
            std::string opt;
            std::string val;

            Unit() {
                parent = NULL;
            }
            Unit(Unit *parent, const char *tag, const char *val = NULL) {
                SP_ASSERT(parent != NULL || tag != NULL);

                this->parent = parent;
                this->tag = tag;
                this->val = (val != NULL) ? val : "";
            }
            Unit(const Unit &unit) {
                copy(unit);
            }
            void copy(const Unit &unit) {
                tag = unit.tag;
                val = unit.val;
                parent = unit.parent;
                child = unit.child;
            }
        };

        Unit *m_crnt;
        MemP<Unit> m_units;

    public:
        JSON() {
            reset();
        }

        void reset() {
            m_units.clear();
            m_crnt = m_units.malloc();
        }

        void nest(const char *tag) {

            SP_ASSERT(tag != NULL);

            Unit *unit = m_units.malloc();
            unit->parent = m_crnt;
            {
                std::string _tag;
                std::string _opt;
                for (int i = 0; tag[i] != 0; i++) {
                    if (tag[i] != ' ') {
                        _tag += tag[i];
                    }
                    else {
                        _opt = &tag[i + 1];
                        break;
                    }
                }

                unit->tag = _tag;
                unit->opt = _opt;
            }

            m_crnt->child.push(unit);
            m_crnt = unit;
        }

        void add(const char *tag, const char *val = NULL) {
            SP_ASSERT(tag != NULL);

            Unit *unit = m_units.malloc();
            unit->parent = m_crnt;

            {
                std::string _tag;
                std::string _opt;
                for (int i = 0; tag[i] != 0; i++) {
                    if (tag[i] != ' ') {
                        _tag += tag[i];
                    }
                    else {
                        _opt = &tag[i + 1];
                        break;
                    }
                }

                unit->tag = _tag;
                unit->opt = _opt;
            }

            if (val != NULL) {
                unit->val = val;
            }

            m_crnt->child.push(unit);
        }

        void unnest() {
            SP_ASSERT(m_crnt != NULL);
            m_crnt = m_crnt->parent;
        }

    public:

        void save(const char *path) {
            File file(path, "w");

            Unit *root = &m_units[0];
            for (int i = 0; i < root->child.size(); i++) {
                save(file, root->child[i], 0);
            }
        }

        void load(const char *path) {
            File file(path, "r");


        }

    private:

        void save(File &file, const Unit *unit, const int level, const bool list = false) {
            char tab[SP_STRMAX] = { 0 };
            for (int i = 0; i < level; i++) {
                strcat(tab, "  ");
            }

            std::string tag0 = unit->tag;
            std::string tag1 = unit->tag;
            if (unit->opt.size() > 0) {
                tag0 += " " + unit->opt;
            }
            std::string val = unit->val;

            if (unit->child.size() == 0) {
                if (unit->val.size() == 0) {
                    file.printf("%s<%s/>\n", tab, tag0.c_str());
                }
                else {
                    file.printf("%s\"%s\":", tab, tag0.c_str());
                    file.printf("%s", val.c_str());
                    if (list == true) {
                        file.printf(",");
                    }
                    file.printf("\n");
                }
            }
            else {
                {
                    file.printf("%s\"%s\":{", tab, tag0.c_str());
                    file.printf("\n");
                    for (int i = 0; i < unit->child.size(); i++) {
                        save(file, unit->child[i], level + 1, i + 1 < unit->child.size());
                    }
                    file.printf("%s}", tab);
                    file.printf("\n");
                }
            }
        }
    };

    class _JSONNest {

    private:
        JSON *json;

    public:
        _JSONNest(JSON &json, const char *tag) {
            this->json = &json;
            nest(tag);
        }
        ~_JSONNest() {
            unnest();
        }
        void nest(const char *tag) {
            json->nest(tag);
        }
        void unnest() {
            json->unnest();
        }
    };

#define JSON_NEST(XML, TAG) _JSON_Nest _nest(JSON, CLASS);
}

#endif