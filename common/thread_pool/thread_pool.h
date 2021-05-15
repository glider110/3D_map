#ifndef _THREAD_POOL_H__
#define _THREAD_POOL_H__

#include <vector>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <iostream>
namespace NS_THREAD_POOL
{
    const int TASKS_MAX_NUM = 10;
    class Thread_pool
    {
        
        public:
            typedef std::function<void()> task_func_t;
            Thread_pool(int size = 5);
            ~Thread_pool();
            bool add_task(const task_func_t&);        //添加任务到队列中，当任务队列中已满返回false
            void stop();                            //终止所有任务，用于退出时释放资源
            bool is_working();
        private:
            typedef std::vector<std::thread*> threads_vector_t;
            typedef std::deque<task_func_t> tasks_deque_t;
            
            void thread_task_handle();
            task_func_t get_task();
            
            int threads_pool_size;
            int tasks_cur_size;             //当前任务缓冲队列中的数量
            threads_vector_t threads_vector;
            tasks_deque_t tasks_deque;
            std::mutex mutex_t;
            std::condition_variable cond_t;
            std::atomic<bool> is_on;

            std::atomic<char> thread_num;   //当前工作的线程数量

    };

}


#endif

