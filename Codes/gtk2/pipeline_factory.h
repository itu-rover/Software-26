#ifndef PIPELINE_FACTORY_H
#define PIPELINE_FACTORY_H

#include "app_data.h"
#include <glib.h>

struct Pipeline {
    char *source;
    char *parser;
    char *decoder;
    char *sink;
};

struct Pipeline* create_pipeline(CustomData *data);
int remote_execute(struct Pipeline *pipeline);

#endif /* PIPELINE_FACTORY_H */
