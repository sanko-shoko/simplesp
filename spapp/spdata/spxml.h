//--------------------------------------------------------------------------------
// Copyright (c) 2017-2020, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SP_XML_H__
#define __SP_XML_H__

#include "spcore/spcore.h"
#include "spapp/spdata/spfile.h"

namespace sp {

    //--------------------------------------------------------------------------------
    // xml
    //--------------------------------------------------------------------------------

    using namespace std;

    class XML {
    
    public:

        struct Unit {
            Unit *parent;
            Mem1<Unit*> child;

            string tag;
            string opt;
            string val;

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
        XML() {
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
                string _tag;
                string _opt;
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
                string _tag;
                string _opt;
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

            file.printf("<?xml version=\"1.0\" encoding=\"utf-8\" standalone=\"yes\"?>\n");

            Unit *root = &m_units[0];
            for (int i = 0; i < root->child.size(); i++) {
                save(file, root->child[i], 0);
            }
        }

        void load(const char *path) {
            File file(path, "r");


        }

    private:

        void save(File &file, const Unit *unit, const int level) {
            char tab[SP_STRMAX] = { 0 };
            for (int i = 0; i < level; i++) {
                strcat(tab, "  ");
            }

            string tag0 = unit->tag;
            string tag1 = unit->tag;
            if (unit->opt.size() > 0) {
                tag0 += " " + unit->opt;
            }
            string val = unit->val;

            if (unit->child.size() == 0) {
                if (unit->val.size() == 0) {
                    file.printf("%s<%s/>\n", tab, tag0.c_str());
                }
                else {
                    file.printf("%s<%s>", tab, tag0.c_str());
                    file.printf("%s", val.c_str());
                    file.printf("</%s>\n", tag1.c_str());
                }
            }
            else {
                {
                    file.printf("%s<%s>", tab, tag0.c_str());
                    file.printf("\n");
                    for (int i = 0; i < unit->child.size(); i++) {
                        save(file, unit->child[i], level + 1);
                    }
                    file.printf("%s</%s>\n", tab, tag1.c_str());
                }
            }
        }
    };

    class _XMLNest {

    private:
        XML *xml;

    public:
        _XMLNest(XML &xml, const char *tag) {
            this->xml = &xml;
            nest(tag);
        }
        ~_XMLNest() {
            unnest();
        }
        void nest(const char *tag) {
            xml->nest(tag);
        }
        void unnest() {
            xml->unnest();
        }
    };

#define XML_NEST(XML, TAG) _XMLNest _nest(XML, TAG);
}

#endif