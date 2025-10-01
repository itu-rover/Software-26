#include "pipeline_factory.h"
#include <stdlib.h>
#include <glib.h>
#include <gtk/gtk.h>

struct Pipeline* create_pipeline(CustomData *data)
{
    if (!data || !data->selected_stream) return NULL;

    char *source = g_strdup_printf("udpsrc port=%d", data->selected_stream->streaming_port);
    char *parser = g_strdup("h264parse");
    char *decoder = g_strdup("avdec_h264");
    char *sink = g_strdup("autovideosink");

    struct Pipeline *pipeline = malloc(sizeof(struct Pipeline));
    if (!pipeline) {
        g_free(source);
        g_free(parser);
        g_free(decoder);
        g_free(sink);
        return NULL;
    }

    pipeline->source = source;
    pipeline->parser = parser;
    pipeline->decoder = decoder;
    pipeline->sink = sink;

    return pipeline;
}

int remote_execute(struct Pipeline *pipeline)
{
    if (!pipeline) return -1;
    // Implementation removed as it had security issues with password handling
    return 0;
} 