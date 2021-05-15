#include "thread_pool.h"

namespace NS_THREAD_POOL
{

    Thread_pool::Thread_pool(int size)
    {
        if(size > 6)
        {
            threads_pool_size = 6;
        }
        else
        {
            threads_pool_size = size;
        }

        threads_vector.reserve(threads_pool_size+1);
        for(int i(0);i < threads_pool_size;i++)
        {
            threads_vector.push_back(new std::thread(std::bind(&NS_THREAD_POOL::Thread_pool::thread_task_handle,this)));
        }
        tasks_cur_size = 0;
        is_on = true;
        thread_num = 0;

    }

    Thread_pool::~Thread_pool()
    {
        if(is_on)
        {
            stop();
        }
    }

    bool Thread_pool::is_working()
    {
        if(thread_num > 0)
        {
            return true;
        }
        return false;
    }

    void Thread_pool::stop()
    {
        {
            std::unique_lock<std::mutex> lock(mutex_t);
            is_on = false;
            cond_t.notify_all();
        }

        for (threads_vector_t::iterator it = threads_vector.begin(); it != threads_vector.end() ; ++it)
        {
            (*it)->join();
            delete *it;
        }
        threads_vector.clear();
    }
    
    void Thread_pool::thread_task_handle()
    {
        while(is_on)
        {
            task_func_t task;
            task = get_task();
            if(task != nullptr)
            {
                task();
                thread_num--;
            }
        }

    }

    bool Thread_pool::add_task(const task_func_t& task_tt)
    {
        std::unique_lock<std::mutex> lock(mutex_t);
        if(tasks_cur_size <= TASKS_MAX_NUM)
        {
            tasks_deque.push_back(task_tt);
            tasks_cur_size++;
            thread_num++;
            cond_t.notify_one();
            return true;

        }
        return false;
    }


    Thread_pool::task_func_t Thread_pool::get_task()
    {
        task_func_t temp;
        std::unique_lock<std::mutex> lock(mutex_t);
        //若任务列表为空，则等待
        while(tasks_deque.empty()&&is_on)
        {
            cond_t.wait(lock);
        }

        //取出一个任务
        if(!tasks_deque.empty() && is_on)
        {
            temp = tasks_deque.front();
            tasks_deque.pop_front();
            tasks_cur_size--;
        }
        

        return temp;
    }



}
