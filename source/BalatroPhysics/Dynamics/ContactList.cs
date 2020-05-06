using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace BalatroPhysics.Dynamics
{

    /// <summary>
    /// A list of contacts with a maximum count of 4(Max constant) elements.
    /// </summary>
    public class ContactList : IList<Contact>
    {
        public const int Max = 4;
        private Contact[] _array;
        private int count;

        public Contact this[int index] 
        {
            get
            {
                if(index < count && index >= 0)
                    return _array[index];
                return null;
            }
            set
            {
                if(index < count && index >= 0)
                    _array[index] = value;
            }
        }

        public int Count
        {
            get
            {
                return count;
            }
        }

        public bool IsReadOnly 
        {
            get
            {
                return false;
            }
        }

        public ContactList()
        {
            _array = new Contact[Max];
            count = 0;
        }

        public void Add(Contact item)
        {
            if(count < Max)
            {
                count++;
                this[count - 1] = item;
            }
        }

        public void Clear()
        {
            Array.Clear(_array, 0, _array.Length);
            count = 0;
        }

        public bool Contains(Contact item)
        {
            for(int i = 0; i < count; i++)
            {
                if (_array[i] == item)
                {
                    return true;
                }
            }
            return false;
        }

        public void CopyTo(Contact[] array, int arrayIndex)
        {
            Array.ConstrainedCopy(_array, 0, array, arrayIndex, count);
        }

        public IEnumerator<Contact> GetEnumerator()
        {
            for(int i = 0; i < count; i++)
            {
                yield return _array[i];
            }
        }

        public int IndexOf(Contact item)
        {
            for (int i = 0; i < count; i++)
            {
                if (_array[i] == item)
                {
                    return i;
                }
            }
            return -1;
        }

        public void Insert(int index, Contact item)
        {
            if(index < Max && index >= 0)
            {
                if (count < Max)
                    count++;

                // Move all elements to right
                if (index < count - 1)
                {
                    for (int i = count - 1; i >= index; i--)
                    {
                        this[i + 1] = this[i];
                    }
                    this[index] = item;
                }
                // Add to end
                else
                    this[count - 1] = item;

            }
        }

        public bool Remove(Contact item)
        {
            for (int i = 0; i < count; i++)
            {
                if (_array[i] == item)
                {
                    RemoveAt(i);
                    return true;
                }
            }
            return false;
        }

        public void RemoveAt(int index)
        {
            if (index < count && index >= 0)
            {
                // Move all elements to left
                for (int i = index; i < count; i++)
                {
                    this[i] = this[i + 1];
                }
                count--;
            }
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
}
