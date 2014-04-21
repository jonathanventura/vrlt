
#ifndef THREADED_H
#define THREADED_H

#include <MultiView/multiview.h>

#if USE_DISPATCH
#include <dispatch/dispatch.h>
#endif

namespace vrlt
{
    class ReconstructionThread
    {
    public:
        virtual ~ReconstructionThread() { }
        
        // parallel part
        virtual void run() { }
        
        // sequential part
        virtual void finish( Reconstruction &r ) { }
    };
    
    struct ThreadInfo
    {
        std::vector<ReconstructionThread*> threads;
    };
    
    static void runThread( void *context, size_t i )
    {
        ThreadInfo *ti = (ThreadInfo*)context;
        ti->threads[i]->run();
    }
    
    static void finishThreads( Reconstruction &r, std::vector<ReconstructionThread*> &threads )
    {
        ThreadInfo ti;
        ti.threads = threads;
        
#if USE_DISPATCH
        dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0);
        dispatch_apply_f( threads.size(), queue, &ti, runThread );
#else
        for ( int i = 0; i < threads.size(); i++ )
        {
            runThread( &ti, i );
        }
#endif
        
        for ( int i = 0; i < threads.size(); i++ )
        {
            threads[i]->finish( r );
            delete threads[i];
        }
        
        threads.clear();
    }
}

#endif
